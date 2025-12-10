import json
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from qdrant_client.http.models import PointStruct, VectorParams, Distance
import uuid
import time
import os

# === CONFIG ===
COHERE_API_KEY = "BFa9b85rwbDFBCNBc6KwK6T49rblQMlEHPJdD0NV"
QDRANT_URL = "https://01d95250-1a34-48b5-92f3-c69429d0c33a.us-east4-0.gcp.cloud.qdrant.io"
QDRANT_API_KEY = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.cdqyrawP2Z3LgTJdvwCqvd74g3ZiHyyjhyhu9hJRvWc"
COLLECTION_NAME = "book-rag"
INPUT_FILE = "backend-RAG/data/book_pages.json" # Adjusted from book_pages_playwright.json

# Initialize clients
cohere = CohereClient(COHERE_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

from qdrant_client.http.exceptions import UnexpectedResponse

# Create collection if not exists
try:
    qdrant.get_collection(COLLECTION_NAME)
    print(f"Collection {COLLECTION_NAME} already exists.")
except UnexpectedResponse: # Or a more specific NotFoundError if available in this version
    qdrant.create_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
    )
    print(f"Collection {COLLECTION_NAME} created")

# Load crawled data
if not os.path.exists(INPUT_FILE):
    print(f"Error: Input file not found at {INPUT_FILE}. Please ensure Step 1 is completed successfully.")
else:
    with open(INPUT_FILE, "r", encoding="utf-8") as f:
        pages = json.load(f)

    print(f"Loaded {len(pages)} pages. Starting chunking & embedding...")

    points = []
    batch_size = 10  # Cohere allows max 96, but we do small batches

    for idx, page in enumerate(pages):
        text = page["text"]
        url = page["url"]
        title = page["title"]

        # Simple chunking: ~400 tokens (~1600 chars)
        chunks = [text[i:i+1600] for i in range(0, len(text), 1400)]

        for chunk_idx, chunk in enumerate(chunks):
            if len(chunk.strip()) < 50:
                continue

            try:
                embedding = cohere.embed(
                    texts=[chunk],
                    model="embed-english-v3.0",
                    input_type="search_document"
                ).embeddings[0]

                point_id = str(uuid.uuid4())
                points.append(PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "url": url,
                        "title": title,
                        "chunk_text": chunk,
                        "chunk_index": chunk_idx
                    }
                ))

                if len(points) >= batch_size:
                    qdrant.upsert(collection_name=COLLECTION_NAME, points=points)
                    print(f"Uploaded batch → total {idx+1}/{len(pages)} pages")
                    points = []
                    time.sleep(0.5)  # Be nice to API

            except Exception as e:
                print(f"Error embedding chunk {chunk_idx} from {url}: {e}")

    # Final batch
    if points:
        qdrant.upsert(collection_name=COLLECTION_NAME, points=points)

    print(f"STEP 2 COMPLETE! All chunks uploaded to Qdrant collection '{COLLECTION_NAME}'")
    print("Ready for Step 3 → bolo Continue to FastAPI endpoint")
