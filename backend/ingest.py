#!/usr/bin/env python3
import asyncio
import logging
import uuid
from datetime import datetime
from xml.etree import ElementTree as ET

import requests
from bs4 import BeautifulSoup
from cohere import Client as CohereClient
from qdrant_client import QdrantClient
from qdrant_client.http.models import Distance, VectorParams, PointStruct
from trafilatura import extract
from langchain_text_splitters import RecursiveCharacterTextSplitter
import tiktoken

from config import config
from utils import clean_text

# Logging setup
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger("ingest")

# Clients
cohere_client = CohereClient(config.COHERE_API_KEY)
qdrant_client = QdrantClient(url=config.QDRANT_URL, api_key=config.QDRANT_API_KEY)

def create_collection():
    try:
        qdrant_client.get_collection(config.COLLECTION_NAME)
        logger.info(f"Collection '{config.COLLECTION_NAME}' already exists.")
    except Exception:
        qdrant_client.create_collection(
            collection_name=config.COLLECTION_NAME,
            vectors_config=VectorParams(size=1024, distance=Distance.COSINE)
        )
        logger.info(f"Created collection '{config.COLLECTION_NAME}'")

def get_urls_from_sitemap():
    logger.info("Fetching sitemap.xml with proper headers...")
    headers = {
        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/128.0.0.0 Safari/537.36"
    }
    try:
        resp = requests.get(config.SITEMAP_URL, headers=headers, timeout=30)
        resp.raise_for_status()
        logger.info(f"Sitemap fetched: {len(resp.content)} bytes")

        # Parse XML with default namespace
        root = ET.fromstring(resp.content)
        namespace = "{http://www.sitemaps.org/schemas/sitemap/0.9}"  # Curly braces add kiye
        urls = []
        for url_elem in root.findall(f'{namespace}url'):
            loc_elem = url_elem.find(f'{namespace}loc')
            if loc_elem is not None and loc_elem.text:
                urls.append(loc_elem.text.strip())

        logger.info(f"Successfully parsed {len(urls)} URLs from sitemap")
        return urls

    except requests.exceptions.RequestException as e:
        logger.error(f"Failed to fetch sitemap: {e}")
        raise
    except ET.ParseError as e:
        logger.error(f"XML parsing error: {e}")
        logger.debug(f"First 1000 chars: {resp.text[:1000]}")
        raise
    except requests.exceptions.RequestException as e:
        logger.error(f"Failed to fetch sitemap: {e}")
        raise
    except ET.ParseError as e:
        logger.error(f"XML parsing error: {e}")
        logger.debug(f"First 1000 chars: {resp.text[:1000]}")
        raise

async def extract_content(url: str) -> dict | None:
    headers = {
        "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36"
    }
    try:
        resp = requests.get(url, headers=headers, timeout=30)
        resp.raise_for_status()
        text = extract(resp.text, favor_precision=True, include_links=False, include_images=False)
        if text and len(text.strip()) > 150:
            title = BeautifulSoup(resp.text, "html.parser").title.string.strip() if BeautifulSoup(resp.text, "html.parser").title else url.split("/")[-1]
            return {
                "url": url,
                "title": title,
                "text": clean_text(text)
            }
        else:
            logger.warning(f"Low content skipped: {url}")
    except Exception as e:
        logger.warning(f"Failed to extract {url}: {e}")
    return None

async def main():
    create_collection()

    # Step 1: Get URLs
    urls = get_urls_from_sitemap()
    if not urls:
        logger.error("No URLs found. Exiting.")
        return

    # Step 2: Extract content from each page
    logger.info("Starting content extraction...")
    pages = []
    for url in urls:
        logger.info(f"Extracting: {url}")
        doc = await extract_content(url)
        if doc:
            pages.append(doc)

    logger.info(f"Successfully extracted content from {len(pages)} pages")

    # Step 3: Chunking + Embedding + Upsert
    encoder = tiktoken.get_encoding("cl100k_base")
    splitter = RecursiveCharacterTextSplitter(
        chunk_size=config.CHUNK_SIZE,
        chunk_overlap=config.CHUNK_OVERLAP,
        length_function=lambda x: len(encoder.encode(x))
    )

    points = []
    total_chunks = 0

    for page in pages:
        chunks = splitter.split_text(page["text"])
        chunks = [c.strip() for c in chunks if len(c.strip()) > 30]

        if not chunks:
            continue

        # Embed in batch
        embeddings = cohere_client.embed(
            texts=chunks,
            model=config.COHERE_MODEL,
            input_type="search_document"
        ).embeddings

        for i, (text, embedding) in enumerate(zip(chunks, embeddings)):
            point_id = str(uuid.uuid4())
            points.append(PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "text": text,
                    "url": page["url"],
                    "title": page["title"],
                    "chunk_index": i,
                    "timestamp": datetime.utcnow().isoformat()
                }
            ))
            total_chunks += 1

        # Batch upsert
        if len(points) >= config.BATCH_SIZE:
            qdrant_client.upsert(collection_name=config.COLLECTION_NAME, points=points)
            logger.info(f"Upserted batch of {len(points)} chunks")
            points = []

    # Final batch
    if points:
        qdrant_client.upsert(collection_name=config.COLLECTION_NAME, points=points)
        logger.info(f"Final upsert: {len(points)} chunks")

    logger.info(f"Ingestion Complete! 🚀 Total chunks stored: {total_chunks}")
    logger.info("You can now start the backend with: uvicorn main.py:app --reload")

if __name__ == "__main__":
    asyncio.run(main())