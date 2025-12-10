import json
import os
import time
from typing import List, Dict, Any
import cohere
from qdrant_client import QdrantClient, models

# --- Credentials (DO NOT PRINT) ---
COHERE_API_KEY = "BFa9b85rwbDFBCNBc6KwK6T49rblQMlEHPJdD0NV"
QDRANT_URL = "https://01d95250-1a34-48b5-92f3-c69429d0c33a.us-east4-0.gcp.cloud.qdrant.io"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.cdqyrawP2Z3LgTJdvwCqvd74g3ZiHyyjhyhu9hJRvWc"
QDRANT_COLLECTION_NAME = "book-rag"

# --- Configuration ---
CHUNK_SIZE = 400
CHUNK_OVERLAP = 50
EMBEDDING_MODEL = "embed-english-v3.0"
VECTOR_SIZE = 1024 # For Cohere v3 embeddings

def chunk_text(text: str, chunk_size: int, chunk_overlap: int) -> List[str]:
    """
    Chunks a given text into smaller pieces with specified overlap.
    A simple word-based chunking is used for token approximation.
    """
    words = text.split()
    chunks = []
    i = 0
    while i < len(words):
        chunk = words[i:i + chunk_size]
        chunks.append(" ".join(chunk))
        i += (chunk_size - chunk_overlap)
        if i < 0: # Handle cases where chunk_overlap > chunk_size, though not expected here
            i = 0
    return chunks

def generate_embeddings(texts: List[str]) -> List[List[float]]:
    """
    Generates Cohere embeddings for a list of texts.
    """
    co = cohere.Client(COHERE_API_KEY)
    try:
        response = co.embed(
            texts=texts,
            model=EMBEDDING_MODEL,
            input_type="search_document"
        )
        return response.embeddings
    except cohere.CohereAPIError as e:
        print(f"Cohere API Error: {e}")
        return []

def upload_to_qdrant(chunks_with_metadata: List[Dict[str, Any]]):
    """
    Uploads chunks and their embeddings to Qdrant.
    """
    client = QdrantClient(
        url=QDRANT_URL, 
        api_key=QDRANT_API_KEY
    )

    # Ensure collection exists and has correct configuration
    client.recreate_collection(
        collection_name=QDRANT_COLLECTION_NAME,
        vectors_config=models.VectorParams(size=VECTOR_SIZE, distance=models.Distance.COSINE),
    )
    print(f"Qdrant collection '{QDRANT_COLLECTION_NAME}' ensured/recreated.")

    # Prepare points for batch upload
    points = []
    texts_to_embed = []
    for i, item in enumerate(chunks_with_metadata):
        texts_to_embed.append(item["chunk_text"])

        # Batch embeddings to reduce API calls
        if (i + 1) % 96 == 0 or (i + 1) == len(chunks_with_metadata): # Cohere free tier limit ~100 texts/req
            print(f"Generating embeddings for {len(texts_to_embed)} chunks...")
            embeddings = generate_embeddings(texts_to_embed)
            
            for j, emb in enumerate(embeddings):
                original_item_index = i - len(texts_to_embed) + 1 + j
                original_item = chunks_with_metadata[original_item_index]
                points.append(
                    models.PointStruct(
                        id=original_item_index, # Simple sequential ID
                        vector=emb,
                        payload={
                            "url": original_item["url"],
                            "title": original_item["title"],
                            "chunk_text": original_item["chunk_text"]
                        },
                    )
                )
            texts_to_embed = [] # Reset for next batch
    
    print(f"Uploading {len(points)} points to Qdrant...")
    operation_info = client.upsert(
        collection_name=QDRANT_COLLECTION_NAME,
        wait=True,
        points=points,
    )
    print(f"Qdrant upsert operation info: {operation_info}")
    return len(points)

def main():
    raw_texts_path = "backend-RAG/data/raw_texts.json"
    
    if not os.path.exists(raw_texts_path):
        print(f"Error: {raw_texts_path} not found. Please ensure Step 1 is completed.")
        return

    with open(raw_texts_path, 'r', encoding='utf-8') as f:
        raw_texts_data = json.load(f)

    all_chunks_with_metadata = []
    for doc in raw_texts_data:
        url = doc.get("url", "N/A")
        title = doc.get("title", "N/A")
        full_text = doc.get("text", "")

        if not full_text:
            print(f"Skipping URL {url} due to empty text.")
            continue

        chunks = chunk_text(full_text, CHUNK_SIZE, CHUNK_OVERLAP)
        for chunk in chunks:
            all_chunks_with_metadata.append({
                "url": url,
                "title": title,
                "chunk_text": chunk
            })
    
    print(f"Total chunks created: {len(all_chunks_with_metadata)}")
    
    if all_chunks_with_metadata:
        uploaded_count = upload_to_qdrant(all_chunks_with_metadata)
        print(f"Step 2 DONE – {uploaded_count} chunks embedded with Cohere and uploaded to Qdrant book-rag collection. Ready for Step 3 → bolo Continue to FastAPI RAG endpoint")
    else:
        print("No chunks to process. Step 2 completed with 0 chunks.")

if __name__ == "__main__":
    main()
