# Specification for RAG Chatbot UI – Physical AI & Humanoid Robotics Textbook
Purpose: Define the exact look, feel, behavior, and integration of the chatbot UI in the Docusaurus frontend. The UI must feel professional, futuristic, and perfectly match the book’s current theme (light: soft blue #E5F3FD accents, dark: deep slate #1e293b with cyan highlights). Use a humanoid robot icon to represent the AI – making it clear the user is chatting with an embodied intelligence expert.

Location
Frontend folder: physical-ai-book/frontend
Files to create:
src/components/RAGChatbot.tsx
src/css/RAGChatbot.css
src/theme/Root.tsx (for global injection)

Core UI Requirements
Floating Launcher Button
Position: Bottom-right corner (20px from edge)
Shape: Circular, 60px diameter
Icon: Humanoid robot (professional, friendly – use SVG or emoji 🤖 with style)
Background: Gradient from --ifm-color-primary to --ifm-color-primary-light
Hover: Slight scale up + shadow
Dark mode: Brighter gradient for visibility

Chat Window
Opens on click, closes on X or outside click
Size: 380px wide, 520px tall
Position: Bottom-right, above launcher
Header: “Physical AI Assistant” + robot icon + close button
Message area: Scrollable, max-height 400px
Input box at bottom with Send button (robot icon on send)
Rounded corners, subtle shadow, border matching theme

Messages Styling
User messages: Right-aligned, blue background (--ifm-color-primary)
AI messages: Left-aligned, gray background (light: #f1f5f9, dark: #334155)
Sources: Small clickable links below AI message
Typing indicator: “Robot is thinking...” with dots animation

Selected Text Feature
Highlight any text in the book
Right-click → context menu item “Ask Physical AI about this”
Opens chat window (if closed) + auto-sends selected text with default question “Explain this:”

Theme Integration
Fully respect current CSS variables (--ifm-color-primary, --ifm-background-surface-color, etc.)
Perfect light/dark mode switch (no custom colors outside variables)
Match navbar/footer style (clean, minimal, professional)

Behavior
API URL: http://localhost:8001/chat (local testing)
Loading state during API call
Error handling with friendly message
Mobile-friendly (responsive)

Validation Checks
Robot icon visible and professional
Matches book theme exactly in both modes
Selected text feature works
Sources are clickable
No layout break on any page

Output Files
frontend/src/components/RAGChatbot.tsx
frontend/src/css/RAGChatbot.css
frontend/src/theme/Root.tsx

Specification complete – UI will look premium, on-brand, and futuristic with humanoid robot personality.
Next Step
Reply: “Continue to /sp.plan for Chatbot UI”