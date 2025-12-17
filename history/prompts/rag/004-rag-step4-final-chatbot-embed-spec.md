# 004 RAG: Step 4 - Final Chatbot Embedding into Docusaurus Specification

## Context
With the RAG backend fully established and functional (Steps 1, 2, and 3), the next critical phase is to integrate a user-facing chatbot interface directly into the Docusaurus frontend. This chatbot will allow users to interact with the RAG system seamlessly while browsing the textbook content.

## Objective
To create a fully working, beautiful, and responsive RAG chatbot component that is globally accessible on every page of the Docusaurus textbook, enabling natural language queries against the RAG system and providing relevant answers with source links.

## Functional Requirements
1.  **Floating Bubble Interface:**
    *   Located in the bottom-right corner of the screen.
    *   Displays as a clickable floating bubble.
    *   Features a visually appealing purple-blue gradient.
2.  **Full Chat Window:**
    *   Clicking the floating bubble expands it into a full chat window.
    *   The chat window should be designed with dark mode in mind, adhering to the project's aesthetic.
    *   Provides an input box for user queries and a send button.
    *   Displays the AI's answer clearly.
    *   Includes clickable source links from the RAG response.
3.  **"Ask AI about this" Feature:**
    *   Allows users to right-click on any selected text within the textbook pages.
    *   A context menu option "Ask AI about this" appears.
    *   Selecting this option automatically sends the highlighted text to the RAG backend as the `selected_text` parameter.

## Technical Implementation Details
### Frontend Component (`src/components/RAGChatBot.tsx`)
*   **Technology:** TypeScript and React.
*   **Structure:** Implemented as a reusable React component (`RAGChatBot.tsx`).
*   **State Management:** Manages its own state for chat history, input, loading status, and window visibility.
*   **API Connection:**
    *   Connects to the FastAPI RAG endpoint at `http://localhost:8001/chat`.
    *   Sends user questions.
    *   If text is selected and "Ask AI about this" is used, sends the selected text in the `selected_text` field of the API request.
    *   Processes and displays the `answer` and `sources` from the API response.
*   **Styling:** Utilizes CSS (potentially CSS Modules or Tailwind CSS, consistent with Docusaurus's setup) for visual presentation.

### Global Injection (`src/theme/Root.tsx`)
*   The `RAGChatBot` component will be injected globally into the Docusaurus application by modifying `src/theme/Root.tsx`. This ensures the chatbot is present on every page.

### Styling (`src/css/custom.css`)
*   **Floating Button Animation:** CSS animations for the floating bubble.
*   **Chat Window Styling:** CSS for the chat window's appearance, including shadow effects and rounded corners.
*   **Dark Mode Support:** Ensures the chat window's styling adapts correctly to the dark mode theme, maintaining consistency with `sp-specify-theme-v1`.

### Docusaurus Configuration (`docusaurus.config.ts`)
*   `docusaurus.config.ts` will be reviewed and updated if necessary to accommodate the new component or any related build configurations.

## Expected Outcome
*   A visually appealing and fully functional RAG chatbot embedded within the Docusaurus frontend.
*   Seamless interaction with the RAG backend for answering user questions.
*   Enhanced user experience through the "Ask AI about this" feature.
*   The project will be 100% complete with the textbook content, a robust RAG backend, and an integrated, interactive chatbot.

## Final Confirmation (Upon Completion)
*   The full code of `src/components/RAGChatBot.tsx` should be shown.
*   A confirmation message: "STEP 4 DONE – CHATBOT LIVE & VISIBLE! Book + RAG Chatbot 100% complete. Run both backends → bolo 'HACKATHON SUBMISSION READY' for final deploy + video guide!"

## References
*   `fastapi-backend-spec.md`
*   `010-sp-specify-theme-spec.md`
*   Previous RAG steps: `001-rag-step1-playwright-crawl-spec.md`, `002-rag-step2-cohere-embed-qdrant-spec.md`, `003-rag-step3-final-fastapi-cohere-only-spec.md`
