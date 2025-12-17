---
id: 0007
title: Updated UI Specification for RAG Chatbot
stage: spec
date: 2025-12-17
surface: agent
model: gemini-1.5-flash
feature: rag-chatbot-ui
branch: 0003-rag-chatbot-ui
user: [USER] # Placeholder, as I don't have this info
command: agan update """Specification for RAG Chatbot UI – Physical AI & Humanoid Robotics Textbook Purpose: Define the exact UI for the RAG chatbot, making it professional, futuristic, and themed around humanoid robotics. The UI must match the book's theme (light: #0284c7 accent, #f8fafc bg, #E5F3FD navbar/footer; dark: #0ea5e9 accent, #0f172a bg, #1e293b navbar/footer). Use a humanoid robot icon (🤖 or SVG) for the assistant to emphasize embodied intelligence. The UI should look like the provided example: floating button, streaming responses, source citations, mobile responsive, dark mode support. Location Frontend folder: physical-ai-book/frontend Files to create (based on example structure): src/components/ChatBot/index.tsx (main export: ChatBot + FloatingButton) src/components/ChatBot/ChatWindow.tsx src/components/ChatBot/ChatWindow.module.css src/components/ChatBot/ChatMessage.tsx src/components/ChatBot/ChatMessage.module.css src/components/ChatBot/ChatInput.tsx src/components/ChatBot/ChatInput.module.css src/components/ChatBot/FloatingChatButton.tsx src/components/ChatBot/FloatingChatButton.module.css src/components/ChatBot/api.ts (backend client) src/theme/Root.js (for global injection) Core UI Requirements Floating Chat Button Bottom-right, 60px circle, gradient from --ifm-color-primary to --ifm-color-primary-light Icon: Humanoid robot (professional 🤖 with animation on hover) Hover: Scale up + shadow Dark mode: Adjust gradient for visibility Chat Window Opens on button click, full overlay on mobile Size: 380px wide, 500px tall (responsive: full-width on mobile) Header: “Physical AI Assistant” + robot icon + close/clear buttons Messages: Scrollable, with user right/blue, AI left/gray Input: Textarea + send button (robot icon on send) Footer: Small text “Powered by Cohere & Qdrant” Messages Styling User: Right, primary color bg AI: Left, emphasis bg, with robot avatar Streaming: Token-by-token + blinking cursor Sources: Below AI message, chips with links (Ch. X, Sec. Y, p.Z or URL) Selected Text Feature Highlight text → right-click “Ask Physical AI about this” → opens chat + sends as selected_text Theme Integration Use all --ifm-* variables Shadows, borders match navbar/footer (subtle in light, deeper in dark) Mobile: Full-screen window, large touch targets Behavior API: Fetch to /chat (local) Streaming support Error handling Clear chat button Validation Checks Matches example UI exactly Robot icon professional & visible Responsive on mobile Dark/light mode perfect Selected text works Specification complete. Next Step Reply: “Continue to /sp.plan for Chatbot UI”"""
labels: ["specification", "ui"]
links:
  spec: specs/0003-rag-chatbot-ui/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/0003-rag-chatbot-ui/spec.md
  - specs/0003-rag-chatbot-ui/checklists/requirements.md
tests: []
---

## Prompt

```text
agan update """Specification for RAG Chatbot UI – Physical AI & Humanoid Robotics Textbook Purpose: Define the exact UI for the RAG chatbot, making it professional, futuristic, and themed around humanoid robotics. The UI must match the book's theme (light: #0284c7 accent, #f8fafc bg, #E5F3FD navbar/footer; dark: #0ea5e9 accent, #0f172a bg, #1e293b navbar/footer). Use a humanoid robot icon (🤖 or SVG) for the assistant to emphasize embodied intelligence. The UI should look like the provided example: floating button, streaming responses, source citations, mobile responsive, dark mode support. Location Frontend folder: physical-ai-book/frontend Files to create (based on example structure): src/components/ChatBot/index.tsx (main export: ChatBot + FloatingButton) src/components/ChatBot/ChatWindow.tsx src/components/ChatBot/ChatWindow.module.css src/components/ChatBot/ChatMessage.tsx src/components/ChatBot/ChatMessage.module.css src/components/ChatBot/ChatInput.tsx src/components/ChatBot/ChatInput.module.css src/components/ChatBot/FloatingChatButton.tsx src/components/ChatBot/FloatingChatButton.module.css src/components/ChatBot/api.ts (backend client) src/theme/Root.js (for global injection) Core UI Requirements Floating Chat Button Bottom-right, 60px circle, gradient from --ifm-color-primary to --ifm-color-primary-light Icon: Humanoid robot (professional 🤖 with animation on hover) Hover: Scale up + shadow Dark mode: Adjust gradient for visibility Chat Window Opens on button click, full overlay on mobile Size: 380px wide, 500px tall (responsive: full-width on mobile) Header: “Physical AI Assistant” + robot icon + close/clear buttons Messages: Scrollable, with user right/blue, AI left/gray Input: Textarea + send button (robot icon on send) Footer: Small text “Powered by Cohere & Qdrant” Messages Styling User: Right, primary color bg AI: Left, emphasis bg, with robot avatar Streaming: Token-by-token + blinking cursor Sources: Below AI message, chips with links (Ch. X, Sec. Y, p.Z or URL) Selected Text Feature Highlight text → right-click “Ask Physical AI about this” → opens chat + sends as selected_text Theme Integration Use all --ifm-* variables Shadows, borders match navbar/footer (subtle in light, deeper in dark) Mobile: Full-screen window, large touch targets Behavior API: Fetch to /chat (local) Streaming support Error handling Clear chat button Validation Checks Matches example UI exactly Robot icon professional & visible Responsive on mobile Dark/light mode perfect Selected text works Specification complete. Next Step Reply: “Continue to /sp.plan for Chatbot UI”"""
```

## Response snapshot

The `specs/0003-rag-chatbot-ui/spec.md` file was created with the user-provided updated UI specification. The `specs/0003-rag-chatbot-ui/checklists/requirements.md` file was created and updated with the validation results. The validation identified several failures related to the technical detail in the UI specification, as it did not align with the principles of a technology-agnostic functional specification for non-technical stakeholders.

## Outcome

- ✅ Impact: The updated UI specification and its associated checklist have been created.
- 🧪 Tests: No tests were run or added for this step.
- 📁 Files: `specs/0003-rag-chatbot-ui/spec.md` and `specs/0003-rag-chatbot-ui/checklists/requirements.md` were created/updated.
- 🔁 Next prompts: The user needs to confirm their intention regarding the level of technical detail in the UI specification.
- 🧠 Reflection: Manual PHR creation was necessary due to persistent `run_shell_command` issues. The strict adherence to the specification quality checklist led to flagging several items, despite the user's detailed input. This highlights a potential mismatch between the user's intent for the spec and the default functional spec checklist, especially for UI components where detailed design often involves technical aspects.

## Evaluation notes (flywheel)

- Failure modes observed: `run_shell_command` with `bash` failed.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Address the `run_shell_command` / PowerShell / bash issue.