# Specification for RAG Chatbot UI – Physical AI & Humanoid Robotics Textbook
Purpose: Define the exact UI for the RAG chatbot, making it professional, futuristic, and themed around humanoid robotics. The UI must match the book's theme (light: #0284c7 accent, #f8fafc bg, #E5F3FD navbar/footer; dark: #0ea5e9 accent, #0f172a bg, #1e293b navbar/footer). Use a humanoid robot icon (🤖 or SVG) for the assistant to emphasize embodied intelligence. The UI should look like the provided example: floating button, streaming responses, source citations, mobile responsive, dark mode support.

Location
Frontend folder: physical-ai-book/frontend
Files to create (based on example structure):
src/components/ChatBot/index.tsx (main export: ChatBot + FloatingButton)
src/components/ChatBot/ChatWindow.tsx
src/components/ChatBot/ChatWindow.module.css
src/components/ChatBot/ChatMessage.tsx
src/components/ChatBot/ChatMessage.module.css
src/components/ChatBot/ChatInput.tsx
src/components/ChatBot/ChatInput.module.css
src/components/ChatBot/FloatingChatButton.tsx
src/components/ChatBot/FloatingChatButton.module.css
src/components/ChatBot/api.ts (backend client)
src/theme/Root.tsx (for global injection)

Core UI Requirements
Floating Chat Button
Bottom-right, 60px circle, gradient from --ifm-color-primary to --ifm-color-primary-light
Icon: Humanoid robot (professional 🤖 with animation on hover)
Hover: Scale up + shadow
Dark mode: Adjust gradient for visibility

Chat Window
Opens on button click, full overlay on mobile
Size: 380px wide, 500px tall (responsive: full-width on mobile)
Header: “Physical AI Assistant” + robot icon + close/clear buttons
Messages: Scrollable, with user right/blue, AI left/gray
Input: Textarea + send button (robot icon on send)
Footer: Small text “Powered by Cohere & Qdrant”

Messages Styling
User: Right, primary color bg
AI: Left, emphasis bg, with robot avatar
Streaming: Token-by-token + blinking cursor
Sources: Below AI message, chips with links (Ch. X, Sec. Y, p.Z or URL)

Selected Text Feature
Highlight text → right-click “Ask Physical AI about this” → opens chat + sends as selected_text

Theme Integration
Use all --ifm-* variables
Shadows, borders match navbar/footer (subtle in light, deeper in dark)
Mobile: Full-screen window, large touch targets

Behavior
API: Fetch to /chat (local)
Streaming support
Error handling
Clear chat button

Validation Checks
Matches example UI exactly
Robot icon professional & visible
Responsive on mobile
Dark/light mode perfect
Selected text works

Output Files
frontend/src/components/ChatBot/index.tsx (main export: ChatBot + FloatingButton)
src/components/ChatBot/ChatWindow.tsx
src/components/ChatBot/ChatWindow.module.css
src/components/ChatBot/ChatMessage.tsx
src/components/ChatBot/ChatMessage.module.css
src/components/ChatBot/ChatInput.tsx
src/components/ChatBot/ChatInput.module.css
src/components/ChatBot/FloatingChatButton.tsx
src/components/ChatBot/FloatingChatButton.module.css
src/components/ChatBot/api.ts (backend client)
src/theme/Root.tsx (for global injection)

Specification complete.
Next Step
Reply: “Continue to /sp.plan for Chatbot UI”