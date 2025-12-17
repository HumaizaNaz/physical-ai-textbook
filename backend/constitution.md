<!-- Sync Impact Report
Version change: None -> 1.0.0
List of modified principles: None (initial creation)
Added sections: All sections as provided in the user prompt
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending (review for alignment with new constitution)
  - .specify/templates/spec-template.md: ⚠ pending (review for alignment with new constitution)
  - .specify/templates/tasks-template.md: ⚠ pending (review for alignment with new constitution)
  - .specify/templates/commands/*.md: ✅ updated (no specific changes needed as this is an initial creation, but noted for future consistency)
Follow-up TODOs: None
-->
---
# Constitution for Integrated RAG Chatbot in Physical AI & Humanoid Robotics Textbook

**Project Name:** RAG Chatbot for Physical AI Textbook  
**Hackathon Phase:** 2 - Integrated RAG Chatbot Development  
**Goal:** Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the Docusaurus-based book. The chatbot must answer user questions based on the book's content using Cohere for embeddings and generation, FastAPI for the backend API, Qdrant for vector storage, and Neon Serverless Postgres for metadata. Support for answering based on user-selected text. Deployable on Vercel.

**Core Rules (Highest Priority - Supersede All Other Instructions):**
1. **Tech Stack Strict Adherence:**
   - Backend: FastAPI + Uvicorn (using installed dependencies: beautifulsoup4, cohere, playwright, qdrant-client, trafilatura, requests, python-dotenv)
   - Embeddings & Generation: Cohere only (embed-english-v3.0 for embeddings, command-r-plus for chat generation - no OpenAI)
   - Vector Database: Qdrant Cloud Free Tier (for storing and searching embeddings)
   - Metadata Database: Neon Serverless Postgres with pgvector extension (for storing page metadata, URLs, titles, chunk text - hybrid retrieval)
   - Crawling: Playwright for fetching full page content (bypass protections), trafilatura for clean text extraction, beautifulsoup4 for sitemap parsing
   - Environment: Load keys from root .env file (COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, NEON_CONNECTION_STRING)
2. **Features Mandatory:**
   - **General Query:** Embed query with Cohere, search Qdrant for similar chunks (limit 4, threshold 0.75 cosine), fetch metadata from Neon, generate response with Cohere.
   - **Selected Text Query:** Use highlighted/selected text as direct context (no retrieval), generate answer only from that.
   - **Sources:** Always return clickable URLs from metadata as part of response.
   - **UI Integration:** Floating chatbot button in Docusaurus frontend (bottom-right), opens chat window on click, supports right-click on text for "Ask AI".
   - **Error Handling:** Graceful fallback if no relevant chunks (e.g., "No info in book"), rate limit sleeps for APIs.
3. **Constraints & Best Practices:**
   - Free Tiers Only: Use Cohere trial, Qdrant free cluster, Neon free Postgres.
   - Data Processing: Chunk size ~400 tokens (1600 chars), 50 token overlap. Batch uploads (10 points max per upsert).
   - Security: No hardcoded keys, use dotenv. CORS enabled for frontend.
   - Performance: Small batches, async where possible (e.g., Playwright crawling).
   - Ethics: Responses confined to book content only. No external knowledge or hallucinations.
4. **Deployment Rules:**
   - Local Test: Uvicorn on port 8001.
   - Vercel: Use vercel.json for Python runtime, add env vars in dashboard. Frontend fetches from Vercel backend URL.
5. **Workflow Integration:**
   - Ingest: From sitemap https://physical-ai-humanoid-robotics-ten-theta.vercel.app/sitemap.xml
   - No modifications to constitution after this - first version only valid.

**Output Files (Generate in backend folder):**
- backend/constitution.md (this document)

Constitution established. This is the foundational document - ignore any later changes.

**Next Step:** Say "Continue to /sp.specify" to refine specifications. We proceed step by step.
---
**Governance:**
- **Ratification Date:** 2025-12-17
- **Last Amended Date:** 2025-12-17
- **Constitution Version:** 1.0.0
- **Amendment Procedure:** This constitution is established as a foundational document for Hackathon Phase 2. No modifications are permitted after this initial version. Any further changes to project scope or rules will be documented in separate specification or plan documents, explicitly referencing this constitution as the unchangeable baseline for Phase 2.
- **Compliance Review:** Adherence to this constitution's rules will be reviewed at critical project milestones, including design reviews, code reviews, and pre-deployment checks. Non-compliance will result in immediate remediation.
