# 003 RAG: Step 3 - Final FastAPI RAG Endpoint (Cohere + Qdrant Only) Specification

## Context
After successfully crawling the textbook content (Step 1) and populating the Qdrant vector database with Cohere embeddings (Step 2), the final step in the RAG pipeline is to expose this functionality via a FastAPI web service. This service will act as a chatbot endpoint, receiving user questions, performing retrieval from Qdrant, and generating answers using Cohere's chat model, sourcing information from the textbook content.

## Objective
To create a fully functional FastAPI application that provides a RAG-powered chatbot endpoint, utilizing the `book-rag` Qdrant collection and Cohere's `command-r-plus` model, ensuring 100% Cohere + Qdrant integration without any OpenAI dependencies.

## Tooling
*   **FastAPI:** Web framework for building the API endpoint.
*   **`uvicorn`:** ASGI server for running the FastAPI application.
*   **`fastapi.middleware.cors.CORSMiddleware`:** For handling Cross-Origin Resource Sharing, allowing the Docusaurus frontend to connect.
*   **`pydantic`:** For data validation and serialization (request and response models).
*   **`cohere` library:** For generating chat responses.
*   **`qdrant-client` library:** For searching the vector database.

## Implementation Details
*   **Location:** The main FastAPI application will be created as `backend_RAG/main.py`.
*   **FastAPI Application Setup:**
    *   `FastAPI` instance initialized with the title "Physical AI Textbook RAG Chatbot".
    *   `CORSMiddleware` configured to allow all origins (`*`) for development and testing.
*   **Client Initialization:**
    *   Cohere client initialized with `COHERE_API_KEY = "BFa9b85rwbDFBCNBc6KwK6T49rblQMlEHPJdD0NV"`.
    *   Qdrant client initialized with `QDRANT_URL` and `QDRANT_API_KEY`.
*   **Request/Response Models:**
    *   `ChatRequest` (Pydantic BaseModel):
        *   `question` (str): The user's query.
        *   `selected_text` (Optional[str]): Optional text selected by the user, to be used as primary context if provided.
    *   `ChatResponse` (Pydantic BaseModel):
        *   `answer` (str): The generated answer.
        *   `sources` (List[str]): URLs of the retrieved sources.
*   **Endpoints:**
    *   **`GET /` (home):** Returns a simple status message: `{"message": "Physical AI Textbook RAG API is LIVE!", "status": "ready"}`.
    *   **`POST /chat`:** The main RAG chatbot endpoint.
        *   **Context Generation:**
            *   If `request.selected_text` is provided and sufficient in length (>20 chars), it is used directly as context.
            *   Otherwise, the user's `question` is embedded using `cohere.embed` (`embed-english-v3.0`, `input_type="search_query"`).
            *   Qdrant is searched (`collection_name="book-rag"`, `limit=4`, `score_threshold=0.7`) with the query embedding.
            *   Retrieved `chunk_text` from Qdrant hits are joined to form the context.
            *   Source URLs are collected.
        *   **Answer Generation:**
            *   Cohere's `chat` API is called with the generated context and the user's `question`.
            *   `model="command-r-plus"`, `temperature=0.1`, `max_tokens=600`.
            *   The prompt instructs Cohere to answer *only* based on the provided textbook content.
        *   **Error Handling:** Catches general exceptions and returns an `HTTPException` with a 500 status code.
*   **Standalone Execution:** The application can be run directly using `uvicorn` if executed as `__main__`, listening on `http://0.0.0.0:8001`.

## Execution and Testing
### Run the FastAPI server (in Terminal 1):
```bash
cd backend_RAG
uvicorn main:app --reload --port 8001
```

### Test the API (in Terminal 2):
```bash
curl -X POST http://localhost:8001/chat \
  -H "Content-Type: application/json" \
  -d '{"question": "What is embodied intelligence?"}'
```

## Expected Outcome
*   The FastAPI server starts successfully and is accessible at `http://localhost:8001`.
*   The `/` endpoint returns the status message.
*   The `/chat` endpoint, when queried, returns a `ChatResponse` containing a relevant `answer` sourced *only* from the textbook content (retrieved from Qdrant) and a list of `sources` (URLs).
*   The answer should be accurate and coherent based on the textbook data.

## References
*   Qdrant Collection: `book-rag`
*   Cohere Embedding Model: `embed-english-v3.0`
*   Cohere Chat Model: `command-r-plus`
*   Previous RAG steps: `001-rag-step1-playwright-crawl-spec.md`, `002-rag-step2-cohere-embed-qdrant-spec.md`
