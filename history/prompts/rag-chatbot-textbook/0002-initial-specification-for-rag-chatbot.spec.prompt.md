---
id: 0002
title: Initial Specification for RAG Chatbot
stage: spec
date: 2025-12-17
surface: agent
model: gemini-1.5-flash
feature: rag-chatbot-textbook
branch: 0001-rag-chatbot-textbook
user: [USER] # Placeholder, as I don't have this info
command: /sp.specify
labels: ["specification"]
links:
  spec: specs/0001-rag-chatbot-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/0001-rag-chatbot-textbook/spec.md
  - specs/0001-rag-chatbot-textbook/checklists/requirements.md
tests: []
---

## Prompt

```text
/sp.specify """sp.specify Specification for RAG Chatbot – Physical AI & Humanoid Robotics Textbook (Updated – No Git/Deployment Mention) Purpose: Refine and validate every detail of the RAG chatbot so that the next steps (/sp.plan, /sp.tasks, /sp.implement) are 100% clear, error-free, and perfectly match the hackathon requirements. This specification is based on the Constitution and focuses only on functionality, data flow, and code structure – no Git, no deployment, no Vercel mentioned here. Project Location Root folder: physical-ai-book Frontend: physical-ai-book/frontend (Docusaurus – already running locally) Backend: physical-ai-book/backend (your folder with pyproject.toml – already ready with all dependencies installed) Live Book URL (for crawling only) https://physical-ai-humanoid-robotics-ten-theta.vercel.app Sitemap: https://physical-ai-humanoid-robotics-ten-theta.vercel.app/sitemap.xml Exact Features (Must Work Exactly Like This) General Question Example: User types “What is embodied intelligence?” → Chatbot searches the book content → returns accurate answer with 1–4 source links (clickable URLs from the book) Selected-Text Question Example: User highlights any paragraph in the book → types a question in chat (or right-click option) → Chatbot answers ONLY using the highlighted text as context (no vector search) → Shows “Answered from selected text” Chatbot Appearance (Frontend Integration – to be implemented later) • Floating blue bubble (bottom-right corner) on every page of the book • Click bubble → chat window opens (dark-mode ready) • Message history, input box, Send button • Sources appear below AI answer as clickable links API Specification (Backend Endpoint) Endpoint: POST /chat Request body (JSON): JSON{ "question": "string", // always required "selected_text": "string" // empty string if no selection } Response (JSON): JSON{ "answer": "string", "sources": ["https://physical-ai-humanoid-robotics-ten-theta.vercel.app/..."] // array of URLs; empty if selected_text used } Data Flow Ingest Process (One-time script) • Parse sitemap to get all page URLs • Use Playwright to fetch full rendered HTML (bypass any protections) • Extract clean readable text with trafilatura • Split text into chunks (~1600 characters, ~200 char overlap) • Embed each chunk with Cohere embed-english-v3.0 (input_type="search_document") • Store vector + payload in Qdrant (collection: rag-book) • Store same data in Neon Postgres table “chunks” for metadata and hybrid retrieval Query Process • If selected_text is provided → use it directly as context • Else → embed question with Cohere embed-english-v3.0 (input_type="search_query") → search Qdrant for top 4 chunks (cosine similarity, threshold 0.75) → fetch corresponding metadata from Neon table → combine chunk_text into context • Send context + question to Cohere command-r-plus with strict instruction: “Answer only using the provided context” • Return answer + source URLs Database Schemas Neon Postgres table (create once): SQLCREATE TABLE IF NOT EXISTS chunks ( id UUID PRIMARY KEY, url TEXT, title TEXT, chunk_text TEXT, embedding VECTOR(1024) ); Qdrant collection: rag-book (vector size 1024, distance COSINE) Files to Create in backend folder ingest.py → complete crawling, chunking, embedding, upload to Qdrant + Neon main.py → FastAPI app with /chat endpoint (and optional /ingest trigger) utils.py (optional) → any helper functions data/ → folder to temporarily save crawled pages JSON specs.md → this file (save it in backend folder) Validation Checks (Must Pass Before Moving Forward) ✓ Uses only your installed dependencies ✓ Loads all keys from root .env (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_CONNECTION_STRING) ✓ Runs locally with uvicorn main:app --reload ✓ General questions return relevant answers with correct source URLs ✓ Selected-text questions use only the provided text ✓ No OpenAI usage anywhere ✓ No external knowledge – answers strictly from book content Output Files backend/specs.md (this exact updated document – copy-paste into your backend folder) Specification updated and fully validated against Constitution. No deployment, Git, or external tools mentioned – purely functional specs. Next Step Reply: “Continue to /sp.plan”"""
```

## Response snapshot

The `specs/0001-rag-chatbot-textbook/spec.md` file was created with the user-provided specification. The `specs/0001-rag-chatbot-textbook/checklists/requirements.md` file was created and updated with the validation results. The validation identified several failures related to the technical detail in the specification, as it did not align with the principles of a technology-agnostic functional specification for non-technical stakeholders.

## Outcome

- ✅ Impact: The feature specification and its associated checklist have been created.
- 🧪 Tests: No tests were run or added for this step.
- 📁 Files: `specs/0001-rag-chatbot-textbook/spec.md` and `specs/0001-rag-chatbot-textbook/checklists/requirements.md` were created/updated.
- 🔁 Next prompts: The user needs to clarify their intention regarding the level of technical detail in the specification.
- 🧠 Reflection: The manual creation of the branch, directory, spec file, and checklist was successful after encountering issues with `create-new-feature.ps1` and `bash` in the `run_shell_command` environment. The strict adherence to the specification quality checklist led to flagging several items, despite the user's detailed input. This highlights a potential mismatch between the user's intent for the spec and the default functional spec checklist.

## Evaluation notes (flywheel)

- Failure modes observed: `run_shell_command` with `bash` and the `create-new-feature.ps1` script failed due to shell parsing issues in the PowerShell environment.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate creating a PowerShell version of `create-phr.sh` and make `create-new-feature.ps1` more robust to various string inputs, or consider alternative ways to pass complex strings to PowerShell scripts.