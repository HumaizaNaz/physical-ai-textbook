from dotenv import load_dotenv
import os

load_dotenv()

class Config:
    COHERE_API_KEY = os.getenv("COHERE_API_KEY")
    QDRANT_URL = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
    SITEMAP_URL = os.getenv("SITEMAP_URL")
    COLLECTION_NAME = os.getenv("COLLECTION_NAME", "physical_ai_book")
    COHERE_MODEL = "embed-english-v3.0"
    CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "400"))
    CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "60"))
    BATCH_SIZE = 64

config = Config()