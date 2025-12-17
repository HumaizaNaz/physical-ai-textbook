---
id: 0003
title: Implementation Plan for RAG Chatbot
stage: plan
date: 2025-12-17
surface: agent
model: gemini-1.5-flash
feature: rag-chatbot-textbook
branch: 0001-rag-chatbot-textbook
user: [USER] # Placeholder, as I don't have this info
command: /sp.plan
labels: ["planning"]
links:
  spec: specs/0001-rag-chatbot-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/0001-rag-chatbot-textbook/plan.md
tests: []
---

## Prompt

```text
/sp.plan """/sp.plan
---
# Implementation Plan for RAG Chatbot – Physical AI & Humanoid Robotics Textbook

**Goal of this Plan:**  
Provide a clear, sequential, step-by-step roadmap to build the full RAG chatbot using only your existing backend folder and installed dependencies. This plan is 100% local-first – we test everything on your machine before any Git or deployment talk.

**Current Status (Already Done):**  
- Frontend Docusaurus running locally (npm run start)  
- Backend folder with pyproject.toml and all dependencies installed  
- Root .env file with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_CONNECTION_STRING  
- Live book URL and sitemap ready for crawling  

**Sequential Plan (Do in This Exact Order)**

1. **Prepare Backend Structure**  
   Time: 2 minutes  
   Actions:  
   - Create folder: `backend/data` (for temporary crawled JSON)  
   - Create empty files: `ingest.py`, `main.py`, `utils.py` (optional)  
   - Confirm .env is in root (physical-ai-book/.env) and accessible

2. **Build & Run Ingest Script (One-Time Data Preparation)**  
   Time: 15–25 minutes (depends on book size)  
   Actions:  
   - Write ingest.py → crawl sitemap with Playwright → extract text with trafilatura → chunk → embed with Cohere → upload to Qdrant + Neon  
   - Run once: `python ingest.py`  
   - Output: Qdrant collection filled, Neon table populated, local JSON saved for debugging  
   - Success sign: Script ends with “Ingest complete!” and no errors

3. **Build FastAPI Backend (Chat Endpoint)**  
   Time: 10 minutes  
   Actions:  
   - Write main.py → FastAPI app with /chat endpoint  
   - Handle both general query and selected_text  
   - Retrieve from Qdrant → fetch metadata from Neon → generate with Cohere  
   - Add optional /ingest endpoint to re-run ingest if needed  
   - Run locally: `uvicorn main:app --reload --port 8001`

4. **Local Testing of Backend**  
   Time: 5 minutes  
   Actions:  
   - Test with curl or Postman:  
     ```bash
     curl -X POST http://localhost:8001/chat \
       -H "Content-Type: application/json" \
       -d '{"question": "What is embodied intelligence?"}'
     ```  
   - Test selected_text:  
     ```bash
     curl -X POST http://localhost:8001/chat \
       -H "Content-Type: application/json" \
       -d '{"question": "Explain this", "selected_text": "Embodied intelligence refers to..."}'
     ```  
   - Success sign: Returns answer + correct sources (or “from selected text”)

5. **Integrate Chatbot into Frontend (Docusaurus)**  
   Time: 15 minutes  
   Actions:  
   - Create React component: `frontend/src/components/RAGChatbot.jsx`  
   - Add CSS for floating bubble and chat window  
   - Implement selected text capture (window.getSelection)  
   - Fetch to http://localhost:8001/chat  
   - Inject component globally via `frontend/src/theme/Root.js`  
   - Test: Open Docusaurus (npm run start) → floating bubble appears → chat works

6. **Final Local End-to-End Test**  
   Time: 5 minutes  
   Actions:  
   - Run backend: `uvicorn main:app --reload --port 8001`  
   - Run frontend: `npm run start`  
   - Ask general questions → get answers with sources  
   - Highlight text → ask question → get answer from selected text only

**Risks & Mitigations**  
- Crawling slow/fails: Use Playwright headless + longer timeouts  
- Cohere rate limits: Add time.sleep(0.5) in batches  
- Neon connection issues: Use psycopg[binary] and exact connection string  
- Chunk overlap: Ensure no data loss in splitting  

**Output Files from This Plan**  
- backend/plan.md (this document – save it)  
- Clear order so you know exactly what to do next

**Plan Complete**  
This is a simple, linear, foolproof plan. No parallel tasks, no confusion.

**Next Step**  
Reply:  
“Continue to /sp.tasks”

```

## Response snapshot

The `specs/0001-rag-chatbot-textbook/plan.md` file was created with the user-provided implementation plan. A Constitution Check was performed, and the plan was found to be fully consistent with the established project constitution. The agent context update step was attempted but failed, with a note that relevant technologies are noted in the conversation.

## Outcome

- ✅ Impact: The implementation plan has been created and validated against the constitution.
- 🧪 Tests: No tests were run or added for this step.
- 📁 Files: `specs/0001-rag-chatbot-textbook/plan.md` was created.
- 🔁 Next prompts: The user is instructed to say "Continue to /sp.tasks".
- 🧠 Reflection: The manual creation of PHRs and skipping of agent context update due to `run_shell_command` limitations is a recurring issue. This workflow should be streamlined if possible (e.g., by providing a more robust PowerShell script or direct tool for agent context management).

## Evaluation notes (flywheel)

- Failure modes observed: `run_shell_command` failed to execute PowerShell script for agent context update.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Provide a direct tool for agent context update or a PowerShell script that can be executed safely by `run_shell_command`.
