# Test Plan

## Context
This Test Plan outlines the strategy and approach for ensuring the quality, accuracy, and functionality of the "physical-ai-textbook" project. It covers testing across the Docusaurus frontend, the FastAPI backend, and the interactive elements that bridge both components. The goal is to provide a robust and reliable learning resource.

## Objectives
*   Verify the accuracy and completeness of textbook content.
*   Confirm the correct rendering and functionality of the Docusaurus frontend across different devices and browsers.
*   Ensure the FastAPI backend endpoints function as specified, providing correct responses and handling errors gracefully.
*   Validate the proper integration and functionality of Co-Learning elements and Live Curl interactions.
*   Ensure adherence to all project-level requirements, including "exactly 3 Co-Learning elements per lesson" and "1 live curl → backend" per chapter.
*   Verify dark mode default and responsiveness.

## Scope of Testing
The testing efforts will cover:
*   **Textbook Content:** Accuracy, clarity, grammar, consistency, and adherence to learning objectives.
*   **Docusaurus Frontend:**
    *   **UI/UX:** Navigation, layout, responsiveness, visual theme (dark mode).
    *   **Component Functionality:** Proper rendering and interaction of custom React components (Co-Learning elements, Live Curl display).
    *   **Static Content:** Correct display of text, images, code blocks.
    *   **Cross-Browser Compatibility:** Testing on major web browsers.
*   **FastAPI Backend:**
    *   **API Endpoints:** Correctness of request/response models, business logic, and error handling.
    *   **Data Persistence:** If applicable, verification of data storage and retrieval.
    *   **Performance:** Basic load testing or performance checks for critical endpoints.
*   **Integration:**
    *   **Frontend-Backend Communication:** Verification that Docusaurus can successfully make requests to and receive responses from FastAPI.
    *   **Co-Learning Element Interaction:** End-to-end testing of all interactive elements.
    *   **Live Curl Execution:** Validation of the in-page `curl` execution and display of results.

## Testing Phases
1.  **Unit Testing:** (Developer-driven)
    *   **Focus:** Individual functions, components (React), FastAPI endpoints, and utility functions.
    *   **Tools:** Python's `pytest` for backend, JavaScript testing frameworks (e.g., Jest, React Testing Library) for frontend.
2.  **Integration Testing:**
    *   **Focus:** Interaction between modules/components (e.g., FastAPI router calling a service, Docusaurus component rendering an external data source).
    *   **Tools:** `pytest` for backend service integrations, end-to-end tests for frontend component integrations.
3.  **End-to-End (E2E) Testing:**
    *   **Focus:** User flows across the entire application (e.g., navigating a chapter, completing a Co-Learning element, running a Live Curl command).
    *   **Tools:** Playwright, Cypress, or similar browser automation tools.
4.  **Manual Testing / Content Review:**
    *   **Focus:** Content accuracy, grammar, UI/UX, responsiveness, and overall user experience.
    *   **Performed by:** Content creators, editors, and quality assurance personnel.

## Test Environment
*   **Development Environment:** Local machines for unit and early integration testing.
*   **Staging Environment:** A pre-production environment replicating the production setup for E2E and manual testing.

## Defect Management
*   Detected defects will be logged, prioritized, and tracked using a suitable issue tracking system.

## Roles and Responsibilities
*   **Developers:** Write unit and integration tests, fix bugs.
*   **Content Creators:** Review content accuracy and provide feedback on UI/UX.
*   **QA/Testers:** Perform integration, E2E, and manual testing.

## References
*   `00-ADR.md`
*   `00-PLAN.md`
*   `docusaurus-platform-spec.md`
*   `fastapi-backend-spec.md`
*   `co-learning-element-spec.md`
*   `live-curl-integration-spec.md`
