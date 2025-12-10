from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from cohere import Client as CohereClient
from qdrant_client import QdrantClient, models
import uvicorn
import json
import yaml
from pathlib import Path
import traceback # Import traceback for detailed error logging

app = FastAPI(title="Physical AI Textbook RAG Chatbot")

# Allow Docusaurus to connect
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Credentials
COHERE_API_KEY = "BFa9b85rwbDFBCNBc6KwK6T49rblQMlEHPJdD0NV"
QDRANT_URL = "https://01d95250-1a34-48b5-92f3-c69429d0c33a.us-east4-0.gcp.cloud.qdrant.io"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.cdqyrawP2Z3LgTJdvwCqvd74g3ZiHyyjhyhu9hJRvWc"
COLLECTION_NAME = "book-rag"

# Initialize clients with longer timeout
cohere_client = CohereClient(COHERE_API_KEY)
qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
    timeout=60.0  # Crucial for stable Qdrant interaction
)

# --- Load Agent and Skill Definitions ---
AGENTS_CONFIG: Dict[str, Any] = {}
SKILLS_CONFIG: Dict[str, Any] = {}

def load_agent_configs():
    global AGENTS_CONFIG
    try:
        config_path = Path(__file__).parent.parent / "frontend" / "public" / "agents-config.json"
        if config_path.exists():
            with open(config_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            AGENTS_CONFIG = {agent["id"]: agent for agent in data.get("agents", [])}
            print(f"Loaded {len(AGENTS_CONFIG)} agent configurations.")
        else:
            print(f"Warning: agents-config.json not found at {config_path}. Using fallback agent.")
            AGENTS_CONFIG = {
                "teaching-assistant": {
                    "id": "teaching-assistant",
                    "name": "Teaching Assistant",
                    "greeting": "Hi! I'm your tutor!",
                    "prompt": "You are a friendly teaching assistant. Answer clearly using book content."
                }
            }
    except Exception as e:
        print(f"Error loading agents-config.json: {e}. Using fallback agent.")
        AGENTS_CONFIG = {"teaching-assistant": {"id": "teaching-assistant", "name": "Teaching Assistant", "greeting": "Hi! I'm your tutor!", "prompt": "You are a friendly teaching assistant. Answer clearly using book content."}}


def load_skill_configs():
    global SKILLS_CONFIG
    skills_dir = Path(__file__).parent.parent / ".claude" / "skills"
    if not skills_dir.is_dir():
        print(f"Error: Skills directory not found at {skills_dir}")
        return

    for skill_dir in skills_dir.iterdir():
        if skill_dir.is_dir():
            skill_md_path = skill_dir / "SKILL.md"
            if skill_md_path.is_file():
                try:
                    content = skill_md_path.read_text(encoding="utf-8")
                    # Split YAML frontmatter from markdown content
                    parts = content.split("---", 2)
                    if len(parts) < 3:
                        print(f"Warning: SKILL.md for {skill_dir.name} is missing YAML frontmatter.")
                        continue
                    
                    frontmatter = yaml.safe_load(parts[1])
                    if frontmatter and "name" in frontmatter:
                        SKILLS_CONFIG[frontmatter["name"]] = frontmatter
                        # Add full markdown content if needed later (not used currently, but good to have)
                        SKILLS_CONFIG[frontmatter["name"]]["full_content"] = parts[2].strip()
                except Exception as e:
                    print(f"Error loading SKILL.md for {skill_dir.name}: {e}")
    print(f"Loaded {len(SKILLS_CONFIG)} skill configurations.")

# Load configs on startup
load_agent_configs()
load_skill_configs()

class ChatRequest(BaseModel):
    question: str
    selected_text: Optional[str] = None
    agent_id: Optional[str] = None # Added agent_id

class ChatResponse(BaseModel):
    answer: str
    sources: List[str] = []

@app.get("/")
def home():
    return {"message": "Physical AI Textbook RAG API is LIVE!", "status": "ready"}

@app.post("/chat", response_model=ChatResponse)
async def rag_chat(request: ChatRequest):
    try:
        # Debug prints
        print(f"DEBUG: AGENTS_CONFIG keys: {AGENTS_CONFIG.keys()}")
        print(f"DEBUG: SKILLS_CONFIG keys: {SKILLS_CONFIG.keys()}")
        print(f"DEBUG: Request agent_id: {request.agent_id}")

        # Choose agent (fallback to teaching-assistant)
        current_agent_id = request.agent_id if request.agent_id and request.agent_id in AGENTS_CONFIG else "teaching-assistant"
        agent_config = AGENTS_CONFIG.get(current_agent_id)
        
        if not agent_config:
            # This should ideally not happen with fallback, but as a safeguard
            raise HTTPException(status_code=400, detail=f"Agent '{current_agent_id}' not found in configurations.")

        context = ""
        sources = []
        base_prompt_instruction = agent_config["prompt"] # Use agent's base prompt

        # If user highlighted text → use only that
        if request.selected_text and len(request.selected_text.strip()) > 20:
            context = request.selected_text.strip()
            sources = ["Selected text from book"]
        else:
            try:
                # Embed query
                emb = cohere_client.embed(
                    texts=[request.question],
                    model="embed-english-v3.0",
                    input_type="search_query"
                ).embeddings[0]

                # Search Qdrant using client.query
                hits = qdrant_client.query(
                    collection_name=COLLECTION_NAME,
                    query=models.Query(
                        query_vector=emb,
                        limit=4,
                        score_threshold=0.7,
                        with_payload=True
                    )
                )

                if hits:
                    context = "\n\n".join([h.payload.get("chunk_text", "") for h in hits])
                    sources = list(set(h.payload.get("url", "Unknown") for h in hits))
                else:
                    context = "No matching content found in the book."
                    sources = []
            except Exception as e:
                context = "Search temporarily unavailable. Please try again later."
                sources = []
                print(f"Qdrant error during search: {e}")
                traceback.print_exc()
        
        # --- Dynamic Prompt Construction based on Agent/Skills ---
        final_prompt_message = ""
        skill_name_triggered = None

        question_lower = request.question.lower()
        base_prompt_lower = base_prompt_instruction.lower()

        # Prioritize skill selection based on agent's prompt hints and keywords in question
        if "robotics-code-generator" in base_prompt_lower or "code" in question_lower or "generate" in question_lower:
            skill_name_triggered = "robotics-code-generator"
        elif "quiz" in base_prompt_lower or "quiz" in question_lower or "mcq" in question_lower:
            skill_name_triggered = "quiz-maker-robotics" # Using quiz-maker-robotics as per latest
        elif "summarize" in base_prompt_lower or "summarize" in question_lower or "overview" in question_lower:
            skill_name_triggered = "chapter-summarizer"
        elif "explain" in base_prompt_lower or "what is" in question_lower or "concept" in question_lower:
            skill_name_triggered = "concept-explainer"
        elif "lesson" in base_prompt_lower or "lesson" in question_lower:
            skill_name_triggered = "lesson-builder"
        
        if skill_name_triggered and skill_name_triggered in SKILLS_CONFIG:
            skill_used = SKILLS_CONFIG.get(skill_name_triggered)
            if skill_used:
                # Dynamically replace placeholder based on skill type
                if "<TASK>" in skill_used["prompt"]:
                    final_prompt_message = skill_used["prompt"].replace("<TASK>", request.question)
                elif "<TEXT>" in skill_used["prompt"]:
                    final_prompt_message = skill_used["prompt"].replace("<TEXT>", context if context != "No matching content found in the book." else request.question)
                elif "<CONCEPT>" in skill_used["prompt"]:
                    final_prompt_message = skill_used["prompt"].replace("<CONCEPT>", request.question)
                
                # Append output format instruction if available
                if skill_used.get("output_format"):
                    final_prompt_message += f"\n\nOutput in {skill_used['output_format']} format."
            else: # Fallback if skill definition is somehow missing after lookup
                 final_prompt_message = f"{base_prompt_instruction}\n\nBook Content:\n{context}\n\nQuestion: {request.question}"
        else:
            # Fallback to general RAG prompt if no specific skill is triggered or skill not found
            final_prompt_message = f"{base_prompt_instruction}\n\nBook Content:\n{context}\n\nQuestion: {request.question}"

        # Generate answer with Cohere
        response = cohere_client.chat(
            message=final_prompt_message,
            model="command-r-plus", # Reverted to command-r-plus, as specified in earlier instructions
            temperature=0.1,
            max_tokens=600
        )

        return ChatResponse(
            answer=response.text.strip(),
            sources=list(set(sources))  # Unique sources
        )

    except Exception as e:
        import traceback
        print(f"ERROR in rag_chat: {e}")
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"RAG error: {str(e)}")

if __name__ == "__main__":
    print("Starting RAG server on http://localhost:8001")
    uvicorn.run(app, host="0.0.0.0", port=8001)
