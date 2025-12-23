from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
import os
from pathlib import Path
import json
import yaml
from cohere import Client as CohereClient
from qdrant_client import QdrantClient

# -------------------- ENV --------------------
load_dotenv()

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

COLLECTION_NAME = "physical_ai_humanoid_robotics_v2"
VECTOR_NAME = "content"

# -------------------- APP --------------------
app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------- CLIENTS --------------------
cohere = CohereClient(COHERE_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

print("Qdrant client loaded:", type(qdrant))
print("Using collection:", COLLECTION_NAME)

# -------------------- LOAD AGENTS & SKILLS --------------------
AGENTS = {}
SKILLS = {}

def load_agents_and_skills():
    global AGENTS, SKILLS

    agents_path = Path("frontend/public/agents-config.json")
    if agents_path.exists():
        with open(agents_path, "r", encoding="utf-8") as f:
            agents_list = json.load(f)
            AGENTS = {a["id"]: a for a in agents_list}
        print(f"Loaded {len(AGENTS)} agents")
    else:
        print("Warning: agents-config.json not found")

    skills_dir = Path(".claude/skills")
    for skill_path in skills_dir.glob("*/SKILL.md"):
        try:
            content = skill_path.read_text(encoding="utf-8")
            parts = content.split("---", 2)
            if len(parts) < 3:
                continue

            meta = yaml.safe_load(parts[1])
            template = parts[2].strip()

            if meta and "name" in meta:
                SKILLS[meta["name"]] = {
                    "name": meta["name"],
                    "description": meta.get("description", ""),
                    "prompt_template": template,
                }
        except Exception as e:
            print(f"Skill load error {skill_path}: {e}")

load_agents_and_skills()

# -------------------- MODELS --------------------
class Query(BaseModel):
    question: str
    selected_text: str = ""
    agent_id: str | None = None

# -------------------- CHAT ENDPOINT --------------------
@app.post("/chat")
def chat(q: Query):
    try:
        current_agent = AGENTS.get(q.agent_id) if q.agent_id else None

        # ---------- DIRECT CONTEXT MODE ----------
        if q.selected_text.strip():
            context_for_llm = q.selected_text
            sources = []
            final_query = q.question or "Summarize the selected text."

        # ---------- RAG MODE ----------
        else:
            # 1. Embed query
            embedding = cohere.embed(
                texts=[q.question],
                model="embed-english-v3.0",
                input_type="search_query",
            ).embeddings[0]

            # 2. Qdrant query (NEW API)
            results = qdrant.query_points(
                collection_name=COLLECTION_NAME,
                query=embedding,
                using=VECTOR_NAME,
                limit=4,
                score_threshold=0.5,
                with_payload=True,
            )

            if not results.points:
                return {"answer": "No relevant data found.", "sources": []}

            context_parts = []
            sources = []

            for point in results.points:
                payload = point.payload or {}
                context_parts.append(payload.get("chunk_text", ""))
                if payload.get("url") and payload["url"] not in sources:
                    sources.append(payload["url"])

            context_for_llm = "\n\n".join(context_parts)
            final_query = q.question

        # ---------- PROMPT BUILD ----------
        if current_agent and current_agent.get("rag_enabled"):
            template = current_agent.get("prompt_template", "")
            message = (
                template.replace("<RAG_TEXT>", context_for_llm)
                        .replace("<QUERY>", final_query)
            )

            if current_agent.get("skills"):
                skill_names = ", ".join(
                    s for s in current_agent["skills"] if s in SKILLS
                )
                message = f"You have access to these skills: {skill_names}\n\n{message}"
        else:
            message = (
                "Answer the question using the context below.\n\n"
                f"{context_for_llm}\n\nQuestion: {final_query}"
            )

        # ---------- COHERE CHAT ----------
        answer = cohere.chat(
            model="command-r-08-2024",
            message=message,
        ).text

        return {"answer": answer, "sources": sources}

    except Exception as e:
        import traceback
        error_detail = f"{e}\n{traceback.format_exc()}"
        print("CHAT ERROR:", error_detail)
        raise HTTPException(status_code=500, detail=error_detail)

print("FastAPI app loaded – /chat endpoint ready")
