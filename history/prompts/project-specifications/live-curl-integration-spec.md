# Live Curl Integration Specification

## Context
As a key interactive feature mandated by `00-ADR.md` and `00-PLAN.md`, each chapter of the "physical-ai-textbook" *must* include "1 live curl → backend" interaction. This document specifies the design, implementation, and presentation of this functionality, allowing learners to directly interact with the FastAPI backend using `curl` commands.

## Core Principles
*   **Direct Interaction:** Provide a direct, runnable `curl` command for learners.
*   **Transparency:** Clearly show both the `curl` command and its output.
*   **Interactivity:** Allow for easy execution (e.g., copy-paste, or an embedded "Run" button).
*   **Backend Validation:** Ensure the `curl` command targets and correctly interacts with the FastAPI backend.
*   **Educational Value:** The `curl` interaction should reinforce concepts taught in the lesson.

## Functionality
The Live Curl Integration feature will:
1.  Display a pre-defined `curl` command.
2.  Allow the user to copy the command easily.
3.  (Optional, but desirable) Provide an in-page mechanism to execute the `curl` command and display its output.

## Implementation Details (Frontend - Docusaurus)
### Presentation
*   **Code Block:** The `curl` command will be presented within a Docusaurus code block, clearly identifiable as a runnable command.
*   **Contextual Information:** Surrounding text will explain the purpose of the `curl` command, what API endpoint it hits, and what to expect in the response.

### Interactive Execution (Proposed)
*   A custom React component will be developed (or integrated) within Docusaurus to provide an "Execute" button adjacent to the `curl` command.
*   Upon clicking "Execute," the component will:
    *   Construct the `fetch` or `axios` equivalent of the `curl` command.
    *   Send a request to the appropriate FastAPI endpoint (as specified in the `curl` command).
    *   Display the JSON response (or other relevant output) in a dedicated area below the command.
    *   Handle and display any network or API errors gracefully.

## Integration Details (Backend - FastAPI)
*   FastAPI endpoints (`fastapi-backend-spec.md`) will be designed to process the requests sent by these live `curl` interactions.
*   CORS headers will be configured in FastAPI to allow requests from the Docusaurus frontend's domain.
*   Each chapter's `curl` command will target a specific, relevant endpoint that demonstrates a concept from that chapter.

## Example Workflow
1.  Lesson explains a concept (e.g., a simple robot movement command).
2.  Textbook displays a `curl` command: `curl -X POST "http://localhost:8000/robot/move" -H "Content-Type: application/json" -d '{"direction": "forward", "distance": 10}'`
3.  User copies and runs the command in their terminal, or clicks an in-page "Execute" button.
4.  FastAPI processes the request, simulating robot movement, and returns a response (e.g., `{"status": "success", "message": "Robot moved forward 10 units"}`).
5.  The response is displayed to the user.

## References
*   `00-ADR.md`
*   `00-PLAN.md`
*   `docusaurus-platform-spec.md`
*   `fastapi-backend-spec.md`
