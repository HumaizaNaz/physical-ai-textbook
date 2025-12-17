from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from dotenv import load_dotenv
import os
from pathlib import Path
import json
import re
import yaml

from cohere import Client as CohereClient
from qdrant_client import QdrantClient
import psycopg2
from psycopg2.extras import RealDictCursor

# Load .env from root (one level up)
load_dotenv(Path(__file__).parent.parent / ".env")

COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
NEON_CONN_STR = os.getenv("NEON_CONNECTION_STRING")

COLLECTION_NAME = "physical_ai_humanoid_robotics"

app = FastAPI()

# CORS for frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Local testing ke liye, baad mein restrict kar dena
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

cohere = CohereClient(COHERE_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

# --- Load Agents and Skills ---
AGENTS = {}
SKILLS = {}

def load_agents_and_skills():
    global AGENTS, SKILLS

    # Load Agents from frontend/public/agents-config.json
    agents_config_path = Path(__file__).parent.parent / "frontend" / "public" / "agents-config.json"
    if agents_config_path.exists():
        with open(agents_config_path, "r", encoding="utf-8") as f:
            agents_list = json.load(f)
            AGENTS = {agent["id"]: agent for agent in agents_list}
        print(f"Loaded {len(AGENTS)} agents.")
    else:
        print(f"Warning: agents-config.json not found at {agents_config_path}")

    # Load Skills from .claude/skills/*/SKILL.md
    skills_dir = Path(__file__).parent.parent / ".claude" / "skills"
    for skill_path in skills_dir.glob("*/SKILL.md"):
        try:
            with open(skill_path, "r", encoding="utf-8") as f:
                content = f.read()
                # Split YAML frontmatter and markdown content
                parts = content.split('---', 2)
                if len(parts) < 3: # Not enough --- delimiters
                    print(f"Warning: Skill file {skill_path} is missing proper YAML frontmatter delimiters.")
                    continue
                
                frontmatter = yaml.safe_load(parts[1])
                skill_prompt_template = parts[2].strip()

                if "name" in frontmatter:
                    skill_id = frontmatter["name"]
                    SKILLS[skill_id] = {
                        "name": frontmatter.get("name"),
                        "description": frontmatter.get("description"),
                        "prompt_template": skill_prompt_template
                    }
                    print(f"Loaded skill: {skill_id}")
                else:
                    print(f"Warning: Skill file {skill_path} is missing 'name' in frontmatter.")
        except Exception as e:
            print(f"Error loading skill from {skill_path}: {e}")
    print(f"Loaded {len(SKILLS)} skills.")

load_agents_and_skills()

class Query(BaseModel):
    question: str
    selected_text: str = ""
    agent_id: str | None = None # Added agent_id

@app.post("/chat")
def chat(q: Query):
    try:
        current_agent = AGENTS.get(q.agent_id) if q.agent_id else None
        
        # Determine base prompt for Cohere
        if q.selected_text.strip():
            # If selected text, prioritize it as context
            context_for_llm = q.selected_text
            sources = [] # No RAG sources if using selected_text directly
            initial_prompt_prefix = "Explain the following text:\n\n<TEXT>\n\n"
            final_user_query = q.question or "Summarize the selected text."
        else:
            # If no selected text, proceed with RAG
            # Embed query
            emb = cohere.embed(
                texts=[q.question],
                model="embed-english-v3.0",
                input_type="search_query"
            ).embeddings[0]

            # Search Qdrant
            results = qdrant.search(
                collection_name=COLLECTION_NAME,
                query_vector=emb,
                limit=4,
                score_threshold=0.5 # Lowered threshold
            )

            if not results:
                return {"answer": "I couldn't find relevant information in the book for your query.", "sources": []}

            # Get chunk texts
            context_parts = []
            sources = []
            for hit in results:
                payload = hit.payload
                context_parts.append(payload["chunk_text"])
                if payload["url"] not in sources:
                    sources.append(payload["url"])

            context_for_llm = "\n\n".join(context_parts)
            initial_prompt_prefix = "" # RAG prompt template handles context
            final_user_query = q.question

        # Construct message for Cohere
        if current_agent and current_agent.get("rag_enabled", False):
            # Use agent's prompt template if RAG is enabled for agent
            prompt_template = current_agent["prompt_template"]
            
            # Replace placeholders in agent's template
            message = prompt_template.replace("<RAG_TEXT>", context_for_llm)
            message = message.replace("<QUERY>", final_user_query)
            message = message.replace("<USER_TASK>", final_user_query) # For robotics mentor

            # Optionally, inject available skills into the prompt
            if current_agent.get("skills"):
                skill_names = ", ".join([SKILLS[s]["name"] for s in current_agent["skills"] if s in SKILLS])
                message = f"You have access to the following skills: {skill_names}.\n\n{message}"
            
        else:
            # Default prompt if no agent or agent.rag_enabled is false
            message = f"Answer the question using only this context from the book:\n\n{context_for_llm}\n\nQuestion: {final_user_query}"

        # If a skill is explicitly invoked within the question (e.g., "summarize X using chapter-summarizer")
        # this part needs more sophisticated parsing. For now, assume agent's prompt handles skill invocation.
        # This basic implementation relies on the LLM understanding which skill to use based on the prompt.

        answer = cohere.chat(
            message=message,
            model="command-a-03-2025"
        ).text

        return {"answer": answer, "sources": sources}

    except Exception as e:
        print(f"Error in chat endpoint: {e}")
        raise HTTPException(status_code=500, detail=f"Sorry, something went wrong: {str(e)}")

# Optional: Re-run ingest
@app.post("/ingest")
def re_ingest():
    return {"answer": "Ingest not triggered from here – run ingest.py manually"}

# Ensure the server runs on port 8001
@app.on_event("startup")
async def startup_event():
    print("FastAPI app loaded – /chat endpoint ready")
    print(f"Agents loaded: {list(AGENTS.keys())}")
    print(f"Skills loaded: {list(SKILLS.keys())}")
    


# To run: uvicorn main:app --host 0.0.0.0 --port 8001 --reload