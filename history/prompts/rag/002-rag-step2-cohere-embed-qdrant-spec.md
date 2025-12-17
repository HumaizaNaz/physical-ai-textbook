# 002 RAG: Step 2 - Chunking, Cohere Embeddings, and Qdrant Upload Specification

## Context
Following the successful crawling of the "physical-ai-textbook" content using Playwright (Step 1), the next stage in the RAG pipeline is to process this raw text data. This involves chunking the text, generating dense vector embeddings using Cohere's API, and then uploading these embeddings along with their metadata to a Qdrant vector database.

## Objective
To transform the crawled textbook content from `book_pages_playwright.json` into a searchable vector index within Qdrant, enabling efficient retrieval of relevant information for RAG purposes.

## Tooling
*   **`json`:** For loading the crawled data.
*   **`cohere` library:** For interacting with the Cohere API to generate text embeddings.
*   **`qdrant-client` library:** For interacting with the Qdrant vector database (creating collections, upserting points).
*   **`uuid`:** For generating unique IDs for Qdrant points.
*   **`time`:** For introducing delays to be respectful to APIs.

## Implementation Details
*   **Location:** The script will be created as `backend_RAG/2_embed_and_upload.py`.
*   **Configuration:**
    *   `COHERE_API_KEY`: Provided as "BFa9b85rwbDFBCNBc6KwK6T49rblQMlEHPJdD0NV".
    *   `QDRANT_URL`: Provided as "https://01d95250-1a34-48b5-92f3-c69429d0c33a.us-east4-0.gcp.cloud.qdrant.io".
    *   `QDRANT_API_KEY`: Provided as "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.cdqyrawP2Z3LgTJdvwCqvd74g3ZiHyyjhyhu9hJRrWc".
    *   `COLLECTION_NAME`: "book-rag".
    *   `INPUT_FILE`: "backend_RAG/data/book_pages_playwright.json".
*   **Qdrant Collection Setup:** The script will check if the specified `COLLECTION_NAME` exists in Qdrant. If not, it will create a new collection with a vector size of 1024 and Cosine distance metric.
*   **Data Loading:** The script loads the JSON data from the `INPUT_FILE`.
*   **Chunking:** Text from each page is chunked into smaller segments (approximately 1400 characters per chunk) to fit embedding model input limits and improve retrieval granularity. Only chunks longer than 50 characters are processed.
*   **Cohere Embedding:** Each text chunk is embedded using Cohere's `embed-english-v3.0` model with `input_type="search_document"`.
*   **Qdrant Upsertion:**
    *   Each embedding, along with its original URL, title, chunk text, and chunk index, is packaged as a `PointStruct`.
    *   Points are batched (size 10) and upserted to the Qdrant collection to optimize API calls.
    *   A small `time.sleep(0.5)` is included between batches to prevent API rate limiting.

## Execution Requirements
The script requires the following Python libraries:
*   `cohere`
*   `qdrant-client`

These will be installed using `pip`. Additionally, `playwright` and `chromium` are required for the previous crawling step (`1_crawl_playwright.py`) to be runnable.

## Usage
The script is executed via the Python interpreter:
```bash
python backend_RAG/2_embed_and_upload.py
```

## Expected Outcome
*   A Qdrant collection named "book-rag" populated with vector embeddings and metadata for all processed text chunks.
*   Prints progress messages for loaded pages and uploaded batches.
*   Upon completion, the message "STEP 2 COMPLETE! All chunks uploaded to Qdrant collection 'book-rag'" will be printed.
*   The agent should then confirm "Continue to FastAPI endpoint".

## References
*   Input File: `backend_RAG/data/book_pages_playwright.json`
*   Qdrant Collection: `book-rag`
*   Cohere Model: `embed-english-v3.0`
