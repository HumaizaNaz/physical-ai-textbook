# Plan for Leveraging Claude Agents in Physical AI Textbook

## Context
A set of specialized Claude agents (Teaching Assistant, Robotics Mentor, Quiz Tutor) is being developed to enhance the user experience and educational value of the "physical-ai-textbook". This plan outlines how these agents will be integrated and utilized to provide personalized support, guidance, and assessment capabilities.

## Objective
To strategically deploy and manage Claude agents to offer dynamic, context-aware assistance to learners, thereby deepening comprehension, facilitating practical application, and improving assessment outcomes across the "physical-ai-textbook."

## Utilization Strategy

### 1. Enhanced Learning Support
*   **Agent(s) Used:** Teaching Assistant, Robotics Mentor
*   **Application:**
    *   **Teaching Assistant:** Provide summaries of chapters, explain complex concepts (using `concept-explainer`), and offer quizzes (`quiz-maker`) for immediate comprehension checks within the textbook interface. Integrate with the RAG system to ensure answers are grounded in the textbook content.
    *   **Robotics Mentor:** Offer practical guidance on robotics code generation (`code-generator-robotics`), explain advanced concepts, and assist with understanding real-world industrial applications. Directly answer "how-to" questions related to ROS 2, Gazebo, Isaac Sim.

### 2. Formative and Summative Assessment
*   **Agent(s) Used:** Quiz Tutor, Teaching Assistant
*   **Application:**
    *   **Quiz Tutor:** Generate targeted quizzes (`quiz-maker`) based on specific chapters or topics, simulate exam scenarios, and provide immediate feedback and explanations. Help students prepare for assessments by identifying knowledge gaps.
    *   **Teaching Assistant:** Offer quick, ad-hoc quizzes on concepts or provide summaries for review, complementing the more structured assessment provided by the Quiz Tutor.

### 3. Content Navigation and Discovery
*   **Agent(s) Used:** Teaching Assistant
*   **Application:**
    *   Assist users in navigating the textbook by summarizing chapters or explaining key terms, helping them quickly find relevant information.

### 4. Integration with Frontend Chatbot
*   **Agent(s) Used:** All agents
*   **Application:**
    *   The RAG chatbot embedded in the Docusaurus frontend will act as the primary interface for interacting with these agents. Users will select which agent they wish to converse with, or the system might dynamically route queries based on content/intent.
    *   Agents will leverage the RAG capabilities to provide answers directly from the textbook content when `rag_enabled` is true.

## Integration Points
*   **Frontend Chatbot Interface:** The `RAGChatBot` component will be extended to allow selection or routing to specific agents.
*   **FastAPI Backend:** The RAG FastAPI endpoint will be modified or extended to route queries to the appropriate agent based on user selection or predefined logic.
*   **Skill Invocation:** Agents will internally invoke the specialized Claude skills (e.g., `chapter-summarizer`, `concept-explainer`, `robotics-code-generator`) based on the user's query and the agent's defined capabilities.

## Deliverables
*   Agent configuration files (`.json`) in both `frontend/public/agents/` and `.claude/agents/`.
*   An enhanced frontend chatbot capable of interacting with multiple agents.
*   Backend logic for agent routing and skill invocation.

## Next Steps
1.  **Create Agent JSON Files:** Implement the agent definition files for Teaching Assistant, Robotics Mentor, and Quiz Tutor.
2.  **Modify RAG Chatbot:** Update the `RAGChatBot` component and FastAPI backend to support agent selection and interaction.
3.  **Refine Agent Prompts:** Continuously review and refine agent personalities and skill usage for optimal interaction.

## References
*   `history/prompts/agents/constitution.md`
*   `history/prompts/skills/constitution.md`
*   `history/prompts/skills/plan.md`
*   `.claude/skills/` directory content
*   `00-PLAN.md`
