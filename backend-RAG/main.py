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
import traceback # Ensure this is at the top
import sys       # Ensure this is at the top
from dotenv import load_dotenv
import os
import re # For regex validation
import ast # For Python code validation

load_dotenv()

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
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = "book-rag"

# Ensure keys are loaded
if not COHERE_API_KEY:
    raise ValueError("COHERE_API_KEY not found in environment variables.")
if not QDRANT_URL:
    raise ValueError("QDRANT_URL not found in environment variables.")
if not QDRANT_API_KEY:
    raise ValueError("QDRANT_API_KEY not found in environment variables.")

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

# --- Quiz Format Validation ---
def validate_quiz_format(text: str) -> bool:
    """
    Validates if the text adheres to the expected MCQ format.
    Checks for:
    - At least one question number (e.g., "1.")
    - At least one option (e.g., "a)")
    - "Correct Answer:" presence
    - "Explanation:" presence
    """
    has_question_number = bool(re.search(r"^\d+\.\s", text, re.MULTILINE))
    has_option = bool(re.search(r"^[a-dA-D]\)", text, re.MULTILINE))
    has_correct_answer = "Correct Answer:" in text
    has_explanation = "Explanation:" in text
    
    return has_question_number and has_option and has_correct_answer and has_explanation

# --- Python Code Validation ---
def validate_python_code(code: str) -> Optional[str]:
    """
    Checks Python code for basic syntax errors.
    Returns error message if invalid, None if valid.
    """
    try:
        # Extract code if it's in a markdown block
        code_match = re.search(r"```python\n(.*?)```", code, re.DOTALL)
        if code_match:
            code_to_check = code_match.group(1).strip()
        else:
            code_to_check = code.strip()

        if not code_to_check:
            return "No Python code found to validate."
        
        ast.parse(code_to_check)
        return None # No syntax errors
    except SyntaxError as e:
        return f"Syntax Error: {e}"
    except Exception as e:
        return f"An unexpected error occurred during code validation: {e}"

# --- FastAPI Endpoints ---
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

                # Search Qdrant using client.search (reverted to previous working method)
                hits = qdrant_client.search(
                    collection_name=COLLECTION_NAME,
                    query_vector=emb,
                    limit=4,
                    score_threshold=0.7,
                    with_payload=True
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
        
        # --- Cohere Generation with Self-Correction ---
        generated_answer = ""
        final_sources = list(set(sources)) # Ensure sources are unique and ready

        retries = 0
        MAX_RETRIES = 2
        
        while retries <= MAX_RETRIES:
            if skill_name_triggered and skill_name_triggered in SKILLS_CONFIG:
                skill_used = SKILLS_CONFIG.get(skill_name_triggered)
                if skill_used:
                    current_prompt_for_llm = ""
                    # Dynamically replace placeholder based on skill type
                    if "<TASK>" in skill_used["prompt"]:
                        current_prompt_for_llm = skill_used["prompt"].replace("<TASK>", request.question)
                    elif "<TEXT>" in skill_used["prompt"]:
                        current_prompt_for_llm = skill_used["prompt"].replace("<TEXT>", context if context != "No matching content found in the book." else request.question)
                    elif "<CONCEPT>" in skill_used["prompt"]:
                        current_prompt_for_llm = skill_used["prompt"].replace("<CONCEPT>", request.question)
                    
                    # Append output format instruction if available
                    if skill_used.get("output_format"):
                        # Special handling for quiz-maker-robotics to make it very explicit
                        if skill_name_triggered == "quiz-maker-robotics":
                            correction_instruction = ""
                            if retries > 0:
                                correction_instruction = "\n\nCRITICAL: Your previous response did NOT meet the required quiz format. You MUST provide 4 MCQs formatted exactly as requested. Ensure each question has 4 options (a,b,c,d), a 'Correct Answer:' line, and an 'Explanation:' line. DO NOT deviate from this structure."
                            
                            current_prompt_for_llm = f"{{{skill_used['prompt'].replace('<TEXT>', context if context != 'No matching content found in the book.' else request.question)}}}{correction_instruction}\n\nPlease format the output as a numbered list of multiple-choice questions.\nFor each question:\n1.  Provide the question text.\n2.  List 4 options, labeled a), b), c), and d).\n3.  Clearly state the \"Correct Answer:\" followed by the letter and option.\n4.  Provide an \"Explanation:\" for the correct answer.\n\nExample Format:\n1. Question text?\n   a) Option A\n   b) Option B\n   c) Option C\n   d) Option D\nCorrect Answer: b) Option B\nExplanation: [Detailed explanation here]\n"
                        elif skill_name_triggered == "robotics-code-generator":
                            # For code generation, add self-correction instruction
                            error_feedback = ""
                            if retries > 0:
                                error_feedback = "\n\nCRITICAL: Your previous code had the following error. Please fix it:\n" + generated_answer # generated_answer holds the previous error
                            current_prompt_for_llm = f"{skill_used['prompt'].replace('<TASK>', request.question)}{error_feedback}" + "\n\nOutput ONLY the corrected code in a markdown block. Do NOT include any explanations outside the code block."
                        else:
                            current_prompt_for_llm += f"\n\nOutput in {skill_used['output_format']} format."
                else: # Fallback if skill definition is somehow missing after lookup
                    current_prompt_for_llm = f"{base_prompt_instruction}\n\nBook Content:\n{context}\n\nQuestion: {request.question}"
            else:
                # Fallback to general RAG prompt if no specific skill is triggered or skill not found
                current_prompt_for_llm = f"{base_prompt_instruction}\n\nBook Content:\n{context}\n\nQuestion: {request.question}"

            # Generate answer with Cohere
            response = cohere_client.chat(
                message=current_prompt_for_llm,
                model="command-r-08-2024", 
                temperature=0.1,
                max_tokens=1500
            )
            generated_answer = response.text.strip()

            # --- Self-Correction Validation ---
            if skill_name_triggered == "quiz-maker-robotics":
                if validate_quiz_format(generated_answer):
                    print(f"DEBUG: Quiz format validated successfully after {retries} retries.")
                    return ChatResponse(answer=generated_answer, sources=final_sources)
                else:
                    print(f"DEBUG: Quiz format validation FAILED. Retrying... (Retry {retries + 1}/{MAX_RETRIES})")
                    retries += 1
                    # If retries are exhausted, break and return the last generated (failed) answer
                    if retries > MAX_RETRIES:
                        print("DEBUG: Max retries for quiz format exceeded.")
                        break # Exit loop to return current (failed) answer
                    # If retrying, generated_answer (from failed attempt) is used as feedback in next loop iteration
            elif skill_name_triggered == "robotics-code-generator":
                code_validation_error = validate_python_code(generated_answer)
                if code_validation_error is None:
                    print(f"DEBUG: Python code validated successfully after {retries} retries.")
                    return ChatResponse(answer=generated_answer, sources=final_sources)
                else:
                    print(f"DEBUG: Python code validation FAILED: {code_validation_error}. Retrying... (Retry {retries + 1}/{MAX_RETRIES})")
                    retries += 1
                    # Prepare for retry: the generated_answer in this case is the code with syntax error,
                    # so we prepend the error to the message for Cohere to fix it.
                    current_prompt_for_llm += f"\n\nCRITICAL: Your previous code had the following error. Please fix it:\n{code_validation_error}\n\nPrevious Code:\n{generated_answer}\n\nOutput ONLY the corrected code in a markdown block. Do NOT include any explanations outside the code block."
                    if retries > MAX_RETRIES:
                        print("DEBUG: Max retries for code validation exceeded. Returning last attempt with error.")
                        generated_answer = f"I tried to generate the code multiple times but encountered errors. Last attempt's error: {code_validation_error}\n\nPrevious Code:\n{generated_answer}"
                        break
            else:
                # For non-quiz, non-code skills, no special format validation needed for now
                return ChatResponse(answer=generated_answer, sources=final_sources)
        
        # If loop finishes (e.g., max retries for quiz or code), return the last generated answer (which might be an error msg)
        return ChatResponse(answer=generated_answer, sources=final_sources)

    except Exception as e:
        import sys # Ensure sys is available
        print(f"ERROR in rag_chat: {e}")
        print(f"DEBUG: Exception details: {sys.exc_info()}")
        raise HTTPException(status_code=500, detail=f"RAG error: {str(e)}")

if __name__ == "__main__":
    print("Starting RAG server on http://localhost:8001")
    uvicorn.run(app, host="0.0.0.0", port=8001)