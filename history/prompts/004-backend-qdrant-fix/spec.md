# Feature Specification: Backend Qdrant Integration Fix

**Feature Branch**: `001-urdu-translation` (backend dependency for this branch)
**Created**: 2025-12-21
**Status**: Completed

## Summary

This specification documents the resolution of critical integration issues with Qdrant and `fastembed` in the backend, ensuring the RAG chatbot can successfully connect to, ingest data into, and query the Qdrant vector database. The primary goal was to fix 500 Internal Server Errors originating from incorrect Qdrant client usage and dependency configuration.

## User Scenarios & Testing

### User Story 1 - Stable RAG Backend Connection (Priority: P1)

**Goal**: The backend API should successfully connect to Qdrant and perform vector search operations without server errors.

**Independent Test**: The backend application starts without Qdrant-related errors, and the `/chat` endpoint responds with relevant information (or appropriate messages) when queried, without returning 500 Internal Server Errors due to Qdrant.

**Acceptance Scenarios**:

1.  **Given** the backend is deployed, **When** a user queries the `/chat` endpoint, **Then** the Qdrant interaction (embedding, searching) completes without `AttributeError`, `MissingArgumentError`, or `UnknownArgumentsError`.
2.  **Given** the `ingest.py` script is run locally, **When** it attempts to create and populate the Qdrant collection, **Then** it completes successfully without `Not existing vector name error`.
3.  **Given** the `ingest.py` script requires `fastembed`, **When** the backend environment is set up, **Then** `fastembed` is installed and available, preventing runtime errors.

## Requirements

### Functional Requirements

-   **FR-BQ-001**: The backend MUST correctly initialize `QdrantClient` and `CohereClient` using environment variables.
-   **FR-BQ-002**: The `/chat` endpoint MUST use the `qdrant.query` method for vector similarity search.
-   **FR-BQ-003**: The `qdrant.query` method MUST correctly accept `query_text` and interpret it using the configured embedding model.
-   **FR-BQ-004**: The `ingest.py` script MUST create the Qdrant collection with a named vector `"fast-bge-small-en"` configured with appropriate `VectorParams`.
-   **FR-BQ-005**: The `ingest.py` script MUST correctly insert `PointStruct` objects into Qdrant, assigning embeddings to the named vector `"fast-bge-small-en"`.
-   **FR-BQ-006**: The backend environment MUST have `fastembed` installed as a dependency.

### Key Entities

-   **Files**: `backend/main.py`, `backend/ingest.py`, `backend/pyproject.toml`
-   **Libraries**: `qdrant-client`, `fastembed`, `cohere`
-   **Qdrant Concepts**: Named Vectors, `QdrantFastembedConfig` (implicitly handled by named vector setup), `PointStruct`

## Success Criteria

### Measurable Outcomes

-   **SC-BQ-001**: The backend `/chat` endpoint returns HTTP 200 OK for RAG queries (when context is found or not found), without 500 Internal Server Errors due to Qdrant.
-   **SC-BQ-002**: `ingest.py` runs to completion without any Qdrant-related errors, successfully populating the configured collection.
-   **SC-BQ-003**: `fastembed` is correctly listed and installed as a dependency, preventing `ImportError` or `ModuleNotFoundError`.
