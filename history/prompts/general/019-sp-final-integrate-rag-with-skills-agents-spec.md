# 019 SP.Final Integrate RAG with Skills & Agents Specification

## Context
This document specifies the final integration step for the "physical-ai-textbook" project, bringing together the RAG system, specialized Claude skills, and defined agents into a cohesive and interactive chatbot embedded within the Docusaurus frontend. This integration marks the completion of the AI-native textbook's core functionality.

## Objective
To fully integrate RAG capabilities, agent personas, and specialized Claude skills within the `RAGChatBot.tsx` component, enabling an intelligent and context-aware conversational experience for users. The chatbot will allow users to select an agent, query the RAG system, and leverage agent-specific skills to generate tailored responses.

## Implementation Details

### 1. Update `RAGChatBot.tsx`
The existing `src/components/RAGChatBot.tsx` will be updated with the following functionalities:
*   **Agent Selector Dropdown:**
    *   A dropdown menu will be added to the chatbot interface, allowing users to select an agent.
    *   The dropdown will list: "Teaching Assistant" (default), "Robotics Code Mentor", and "Quiz Coach".
*   **RAG Retrieval Integration:**
    *   When a user asks a question, the `RAGChatBot.tsx` will first retrieve relevant textbook content using the RAG backend (`http://localhost:8001/chat`). This content will serve as grounding for the agent's response.
*   **Agent Prompt and Skill Invocation:**
    *   The selected agent's `prompt` (from `agents-config.json`) will be used to prepend the user's question, guiding the AI's response generation.
    *   The agent's specified `skills` will be invoked by the backend based on the query and RAG context (e.g., if a user asks for code, the `robotics-code-generator` skill might be called).
*   **Final Answer Display + Sources:**
    *   The final answer from the RAG backend (which incorporates agent persona and skill output) will be displayed.
    *   Clickable source links (URLs from the RAG retrieval) will be shown alongside the answer.

### 2. Create `public/agents-config.json`
A new JSON file will be created at `frontend/public/agents-config.json` to provide configuration for the frontend to easily load agent definitions.

```json
{
  "agents": [
    {
      "id": "teaching-assistant",
      "name": "Teaching Assistant",
      "greeting": "Hi! I'm your Physical AI tutor. Ask me anything!",
      "prompt": "You are a friendly teaching assistant. Use retrieved book content and skills: concept-explainer, chapter-summarizer, lesson-formatter"
    },
    {
      "id": "robotics-mentor",
      "name": "Robotics Code Mentor",
      "greeting": "I'm your robotics code expert from Figure.ai. Need ROS2/Gazebo/Isaac code?",
      "prompt": "You are a senior robotics engineer. Always use robotics-code-generator skill when code is needed."
    },
    {
      "id": "quiz-coach",
      "name": "Quiz Coach",
      "greeting": "Ready for a quiz? I'll test your Physical AI knowledge!",
      "prompt": "You are a strict quiz master. Always use quiz-maker-robotics skill to generate 4 MCQs with explanations."
    }
  ]
}
```

### 3. Chatbot Features
*   **Floating Bubble:** (Already implemented as per `sp.rag-step4-final-chatbot-embed-spec.md`).
*   **Agent Selector Dropdown:** As described above, integrated into the UI.
*   **Selected Text Query:** Users can right-click on selected text and choose "Ask [Agent Name]", which sends the selected text to the backend.
*   **Clickable Sources:** Retrieved source URLs from the RAG backend will be displayed as clickable links.
*   **Dark Mode Perfect:** The entire chatbot interface, including the agent selector and chat window, will maintain perfect dark mode compatibility, consistent with the `sp.specify-theme-v1`.

### 4. Connect to Backend
The chatbot will continue to connect to the FastAPI RAG endpoint at `http://localhost:8001/chat`.
*   Normal questions will be sent as `{"question": "..."}`.
*   Queries from selected text will be sent as `{"question": "...", "selected_text": "..."}`.

### 5. `docusaurus.config.ts` Updates
Review and update `docusaurus.config.ts` as necessary to support any new components, scripts, or global configurations related to the chatbot and agent integration.

## Expected Outcome
A fully integrated, AI-native textbook experience where users can interact with specialized agents, receive RAG-grounded answers, and leverage AI skills for learning, coding, and assessment, all within a beautiful and responsive Docusaurus frontend.

## Final Confirmation (Upon Completion)
"FINAL INTEGRATION COMPLETE! RAG + 5 Skills + 3 Agents + Agent Switching + Selected Text Query = 100% AI-NATIVE TEXTBOOK! Hackathon submission ready — +150 bonus points locked!"
