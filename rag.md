/sp.rag-step1-crawl-and-clean
---
You are my senior RAG engineer.

Folder: backend-RAG

Do these things right now:

1. Create file: backend-RAG/crawl_and_clean.py
2. Use requests + trafilatura to:
   - Download sitemap: https://physical-ai-textbook-five.vercel.app/sitemap.xml
   - Extract all <loc> URLs
   - For every URL → requests.get() → trafilatura.extract() → get clean readable text only
   - Skip PDFs, images, non-text pages
3. Save result as JSON:
   backend-RAG/data/raw_texts.json
   Format:
   [
     {"url": "https://.../intro", "title": "Foundations...", "text": "long clean text..."},
     ...
   ]

Run the script automatically and when finished say:
“Step 1 DONE – X pages crawled and cleaned. Ready for Step 2 

its step 2 
/sp.rag-step2-embed-and-upload
---
You are my senior RAG engineer.

Folder: backend_RAG

Now do Step 2 using these credentials (keep them secret, never print them):

Cohere API Key: BFa9b85rwbDFBCNBc6KwK6T49rblQMlEHPJdD0NV
Qdrant URL: https://01d95250-1a34-48b5-92f3-c69429d0c33a.us-east4-0.gcp.cloud.qdrant.io
Qdrant API Key: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.cdqyrawP2Z3LgTJdvwCqvd74g3ZiHyyjhyhu9hJRvWc
Qdrant Collection Name: book-rag

Tasks:
1. Read backend_RAG/data/raw_texts.json (from Step 1)
2. Chunk every text into ~400-token chunks (overlap 50 tokens)
3. Use Cohere embed-english-v3.0 model to generate embeddings
4. Upload all chunks + metadata to Qdrant collection "book-rag"
   - vector size = 1024 (Cohere v3)
   - payload: {"url": "...", "title": "...", "chunk_text": "..."}
5. Use batch upload for speed

Create file: backend_RAG/embed_and_upload.py
Run it automatically.

When finished say:
“Step 2 DONE – X chunks embedded with Cohere and uploaded to Qdrant book-rag collection.
Ready for Step 3 → bolo Continue to FastAPI RAG endpoint”


this is step 3 
/sp.rag-step3-fastapi-endpoint
---
You are my senior RAG engineer.

Folder: backend_RAG

Step 3: Create the RAG FastAPI endpoint using Cohere for generation.

Do these things:

1. Create backend_RAG/main.py (FastAPI app with CORS for Docusaurus)
2. Add endpoint: POST /chat
   - Input: {"query": "user question", "selected_text": "optional highlighted text"}
   - Logic:
     - If selected_text exists → use it as context, skip retrieval
     - Else → query Qdrant for top 3 chunks (similarity > 0.7)
     - Combine context + query → Cohere generate (model: command-r-plus, prompt: "Answer based on this book content: [context]")
     - Output: {"answer": "response", "sources": [urls]}
3. Use these credentials (keep secret):
   Cohere API: BFa9b85rwbDFBCNBc6KwK6T49rblQMlEHPJdD0NV
   Qdrant URL: https://01d95250-1a34-48b5-92f3-c69429d0c33a.us-east4-0.gcp.cloud.qdrant.io
   Qdrant API: eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.cdqyrawP2Z3LgTJdvwCqvd74g3ZiHyyjhyhu9hJRvWc
   Collection: book-rag

4. Add uvicorn run command: uvicorn main:app --reload --port 8001

5. Test it: Create test_chat.py with sample query "What is ROS 2?" and run to show output

When finished say:
"Step 3 DONE – FastAPI RAG endpoint ready at http://localhost:8001/chat. Test passed. Ready for Step 4 → bolo Continue to Docusaurus embed"
