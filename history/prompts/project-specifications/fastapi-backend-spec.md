# FastAPI Backend Specification

## Context
This document specifies the core usage, architecture, and principles for the FastAPI backend supporting the "physical-ai-textbook" project. The backend's primary role is to provide interactive capabilities, support live demos, and facilitate potential grading mechanisms, primarily through API endpoints consumed by the Docusaurus frontend.

## Core Principles
*   **Lightweight & Performant:** FastAPI's asynchronous nature and high performance are critical for responsive interactive elements.
*   **API-First Design:** All interactions with the frontend will be via clearly defined RESTful API endpoints.
*   **Type Safety & Validation:** Pydantic models will be extensively used for request and response validation, ensuring data integrity.
*   **Modularity:** The backend will be structured into logical modules (e.g., authentication, chapter-specific logic, data handling).
*   **Security:** Implement robust authentication and authorization mechanisms (if required for grading or personalized content).

## Key Components & Structure
### Application Entry Point (`main.py`)
*   Initialization of the FastAPI application.
*   Mounting of API routers from different modules.
*   Configuration of CORS (Cross-Origin Resource Sharing) to allow requests from the Docusaurus frontend.

### API Routers
*   **Authentication Router:** Endpoints for user authentication (e.g., login, logout, token generation if applicable).
*   **Chapter-Specific Routers:** Dedicated routers for each textbook chapter that requires backend interaction. These will house the logic and endpoints for "live demos" and interactive exercises.
*   **Utility Routers:** General-purpose endpoints (e.g., health checks, global data fetching).

### Data Handling
*   **Pydantic Models:** Used for defining request bodies, response models, and internal data structures, ensuring clear data contracts.
*   **Data Persistence:** (To be detailed in specific ADRs or further specifications) - This could involve in-memory storage for simple demos, or integration with databases for more complex scenarios (e.g., user progress, grading results).

### Dependencies
*   **`uvicorn`:** ASGI server for running the FastAPI application.
*   **Standard Python Libraries:** For general-purpose tasks.
*   **(Potential) External Libraries:** For specific functionalities like data processing, AI model serving, or database interaction (will be documented in specific module specs).

## Integration Points
*   **Docusaurus Frontend:** Consumes the API endpoints to render interactive elements and provide dynamic content.
*   **External Tools/Simulators:** FastAPI endpoints may orchestrate or interact with external simulation environments (e.g., Isaac Sim, Gazebo) for live demos.

## API Endpoint Guidelines
*   **RESTful Naming:** Use clear, resource-oriented endpoint naming conventions (e.g., `/chapters/{chapter_id}/exercise/{exercise_id}/submit`).
*   **Status Codes:** Utilize appropriate HTTP status codes for responses (200 OK, 201 Created, 400 Bad Request, 401 Unauthorized, 404 Not Found, 500 Internal Server Error).
*   **Error Handling:** Consistent error response format across all APIs.

## References
*   `backend/main.py`
*   `00-ADR.md`
*   `00-PLAN.md`
*   `docusaurus-platform-spec.md`
