# Architectural Decision Record: Qdrant Integration Strategy for Named Vectors and `query_text`

**Status**: Accepted
**Date**: 2025-12-21

## Context

During the development and debugging of the RAG chatbot's backend, several 500 Internal Server Errors were encountered related to Qdrant client interactions. These errors manifested as:
1.  `AttributeError: 'QdrantClient' object has no attribute 'search'`
2.  `QdrantFastembedMixin.query() missing 1 required positional argument: 'query_text'`
3.  `Unknown arguments: ['query_vector']`
4.  `Wrong input: Not existing vector name error: fast-bge-small-en` (during ingestion)

The `qdrant-client` library (`>=1.16.2`) introduced changes in its API, particularly regarding the `query` method and how it interacts with embedding models, especially when using `query_text` for internal embedding.

## Decision

The decision is to align the Qdrant integration with the `qdrant-client>=1.16.2` API by:

1.  **Using `qdrant.query` for vector search**: This is the correct method for performing vector similarity searches in the current client version.
2.  **Providing `query_text` to `qdrant.query`**: When using `query_text`, the Qdrant client handles the embedding internally.
3.  **Configuring Qdrant collection with a named vector**: The collection (`physical_ai_humanoid_robotics`) must explicitly define a named vector, `"fast-bge-small-en"`, with its `VectorParams` (size=1024, distance=COSINE). This informs Qdrant about the embedding model to use when `query_text` is provided.
4.  **Assigning embeddings to the named vector during ingestion**: `PointStruct` objects must explicitly assign the generated embeddings to the `"fast-bge-small-en"` named vector using a dictionary format (`"vector": {"fast-bge-small-en": embedding}`).
5.  **Adding `fastembed` as a dependency**: Ensure `fastembed` is installed in the backend environment as it is used by `qdrant-client` for internal embedding when `query_text` is utilized.

## Rationale

*   **API Alignment**: Directly addresses the `AttributeError` and argument-related errors by conforming to the current `qdrant-client` API.
*   **Clear Configuration**: Explicitly defining the named vector `"fast-bge-small-en"` in the collection creation resolves the "Not existing vector name error", ensuring Qdrant understands which embedding model to use for internal embedding of `query_text`.
*   **Correct Data Ingestion**: Assigning embeddings to the named vector in `PointStruct` ensures the data is stored in the collection structure expected by the `qdrant.query` method.
*   **Dependency Resolution**: Installing `fastembed` prevents runtime errors when Qdrant attempts to use it for internal embedding.
*   **Maintainability**: This approach uses well-defined parameters for Qdrant interaction, making future debugging and updates easier.

## Consequences

*   **Requires `ingest.py` Rerun**: The Qdrant collection must be re-created (by running `ingest.py` locally) to apply the new named vector configuration.
*   **Backend Redeployment**: The updated `main.py`, `ingest.py`, and `pyproject.toml` must be redeployed to the Hugging Face Space.
*   **Deprecation Warning**: The `qdrant-client` might still issue a `UserWarning` about the `query` method being deprecated in favor of `query_points`. This is noted but does not currently impede functionality and can be addressed in a future refactor.

## Alternatives Considered

*   **Downgrading `qdrant-client`**: Rejected, as it would mean missing out on new features and fixes in newer versions.
*   **Client-side embedding before Qdrant search**: While possible, the `query_text` approach with named vectors offloads embedding to Qdrant, simplifying client-side logic in `main.py` if correctly configured. The current approach leverages `qdrant-client`'s internal embedding capabilities, which is often more efficient.
