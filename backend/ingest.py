# backend/ingest.py
import asyncio
import json
import os
import time
import uuid
from pathlib import Path

import requests
from bs4 import BeautifulSoup
from cohere import Client as CohereClient
from dotenv import load_dotenv
from playwright.async_api import async_playwright
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct, CollectionStatus
from qdrant_client.http.exceptions import UnexpectedResponse
from trafilatura import extract
import psycopg2
from psycopg2.extras import execute_values

# Load .env from root directory (one level up)
load_dotenv(dotenv_path=Path(__file__).parent.parent / ".env")

# Keys from .env
COHERE_API_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
NEON_CONN_STR = os.getenv("NEON_CONNECTION_STRING")

# Constants
SITEMAP_URL = "https://physical-ai-humanoid-robotics-ten-theta.vercel.app/sitemap.xml"
DATA_DIR = Path("data")
DATA_DIR.mkdir(exist_ok=True)
JSON_OUTPUT = DATA_DIR / "book_pages.json"
COLLECTION_NAME = "physical_ai_humanoid_robotics"

# Clients
cohere = CohereClient(COHERE_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)
conn = psycopg2.connect(NEON_CONN_STR)
cur = conn.cursor()

# Create Neon table
cur.execute("""
CREATE EXTENSION IF NOT EXISTS vector;
CREATE TABLE IF NOT EXISTS chunks (
    id UUID PRIMARY KEY,
    url TEXT,
    title TEXT,
    chunk_text TEXT,
    embedding VECTOR(1024)
);
""")
conn.commit()

# Create Qdrant collection if not exists
# Create Qdrant collection if not exists
try:
    qdrant.get_collection(collection_name=COLLECTION_NAME)
    print(f"Collection '{COLLECTION_NAME}' already exists.")
except UnexpectedResponse as e:
    if e.status_code == 404: # Collection not found
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=VectorParams(size=1024, distance=Distance.COSINE),
        )
        print(f"Created Qdrant collection: {COLLECTION_NAME}")
    else:
        raise e # Re-raise if it's another type of error

async def crawl_pages():
    # Get URLs from sitemap
    resp = requests.get(SITEMAP_URL)
    soup = BeautifulSoup(resp.content, "xml")
    urls = [loc.text.strip() for loc in soup.find_all("loc")]
    print(f"Found {len(urls)} URLs in sitemap")

    pages = []
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)
        context = await browser.new_context(
            user_agent="Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36"
        )
        page = await context.new_page()

        for url in urls:
            try:
                print(f"Crawling: {url}")
                await page.goto(url, wait_until="domcontentloaded", timeout=60000)
                await page.wait_for_timeout(3000)  # Let JS load
                html = await page.content()
                text = extract(html)
                title = await page.title()
                if text and len(text.strip()) > 100:
                    pages.append({"url": url, "title": title, "text": text.strip()})
                    print(f"Success: {title[:60]}...")
                else:
                    print(f"Skipped (low content): {url}")
            except Exception as e:
                print(f"Failed {url}: {e}")
        await browser.close()

    # Save raw pages
    with open(JSON_OUTPUT, "w", encoding="utf-8") as f:
        json.dump(pages, f, indent=2, ensure_ascii=False)
    print(f"Saved {len(pages)} pages to {JSON_OUTPUT}")
    return pages

def chunk_text(text: str, chunk_size: int = 1600, overlap: int = 200):
    chunks = []
    start = 0
    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]
        chunks.append(chunk)
        start = end - overlap
        if end >= len(text):
            break
    return chunks

def ingest_to_databases(pages):
    points = []
    neon_rows = []
    batch_size = 8 # Changed from 10 to 8 as per new prompt

    for page in pages:
        chunks = chunk_text(page["text"])
        for chunk in chunks:
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
                        "url": page["url"],
                        "title": page["title"],
                        "chunk_text": chunk
                    }
                ))

                neon_rows.append((point_id, page["url"], page["title"], chunk, embedding))

                if len(points) >= batch_size:
                    qdrant.upsert(collection_name=COLLECTION_NAME, points=points)
                    execute_values(cur, """
                        INSERT INTO chunks (id, url, title, chunk_text, embedding)
                        VALUES %s
                    """, [(row[0], row[1], row[2], row[3], row[4]) for row in neon_rows])
                    conn.commit()
                    print(f"Uploaded batch of {batch_size} chunks")
                    points = []
                    neon_rows = []
                    time.sleep(0.6)  # Changed from 0.5 to 0.6 as per new prompt
            except Exception as e:
                print(f"Embedding error: {e}")

    # Final batch
    if points:
        qdrant.upsert(collection_name=COLLECTION_NAME, points=points)
        execute_values(cur, """
            INSERT INTO chunks (id, url, title, chunk_text, embedding)
            VALUES %s
        """, [(row[0], row[1], row[2], row[3], row[4]) for row in neon_rows])
        conn.commit()
        print(f"Final batch uploaded. Total chunks processed.")

    cur.close()
    conn.close()
    print("Ingest complete! All data in Qdrant and Neon.")

async def main():
    print("Starting crawl...")
    pages = await crawl_pages()
    print("Starting embedding & upload...")
    ingest_to_databases(pages)

if __name__ == "__main__":
    asyncio.run(main())
