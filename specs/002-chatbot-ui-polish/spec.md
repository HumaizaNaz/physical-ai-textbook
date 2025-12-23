# Feature Specification: Chatbot UI Polish

**Feature Branch**: `001-urdu-translation` (part of polish for this branch)
**Created**: 2025-12-21
**Status**: Completed

## Summary

This specification documents the UI/UX enhancements applied to the RAG chatbot to improve its appearance, positioning, and overall user experience, ensuring it integrates well with the Docusaurus theme while maintaining a distinct, contrasting look.

## User Scenarios & Testing

### User Story 1 - Chatbot Positioning & Icon (Priority: P1)

**Goal**: The user wants the chatbot to be unobtrusive when closed and easily accessible when open, with an intuitive icon.

**Independent Test**: The chatbot button is visible in the bottom-right corner. Clicking it opens a chat window also in the bottom-right. The icon visually represents 'chat' and the send button looks like a 'send' action.

**Acceptance Scenarios**:

1.  **Given** the user is on any page, **When** the page loads, **Then** a chat button with a '🗨️' icon is visible at the bottom-right of the screen.
2.  **Given** the chat button is visible, **When** the user clicks the button, **Then** a chat window appears in the bottom-right of the screen, and the button icon changes to '✕'.
3.  **Given** the chat window is open, **When** the user clicks the '✕' button, **Then** the chat window closes, and the button icon changes back to '🗨️'.
4.  **Given** the chat window is open, **When** the user types a message, **Then** the send button with a '➤' icon is clickable.

### User Story 2 - Chatbot Theming & Readability (Priority: P1)

**Goal**: The chatbot's UI should have a distinct, contrasting theme that is aesthetically pleasing and ensures readability, adapting to light and dark modes.

**Independent Test**: The chatbot's colors (header, background, message bubbles, text) consistently apply the defined contrasting theme and adapt correctly to Docusaurus light/dark mode changes. User messages have a blue/dark gray text.

**Acceptance Scenarios**:

1.  **Given** the Docusaurus theme is in light mode, **When** the chatbot is open, **Then** the chatbot header displays an emerald green/teal gradient, message bubbles use the defined contrast colors, and the icon on the floating button is `#1e293b`.
2.  **Given** the Docusaurus theme is in dark mode, **When** the chatbot is open, **Then** the chatbot header displays a darker emerald green/teal gradient, message bubbles use the defined contrast colors for dark mode, and the icon on the floating button is `white`.
3.  **Given** the chatbot is open, **When** a user sends a message, **Then** the text within their message bubble is blue/dark gray (in light mode) or light gray/white (in dark mode).
4.  **Given** the chatbot is open, **When** the user types, **Then** the input field and send button colors match the defined chatbot theme.

## Requirements

### Functional Requirements

-   **FR-CB-001**: The chatbot MUST display a floating button in the bottom-right corner.
-   **FR-CB-002**: The floating button MUST display a '🗨️' icon when closed and '✕' when open.
-   **FR-CB-003**: The chatbot window MUST open and close from the bottom-right corner of the screen.
-   **FR-CB-004**: The send message button MUST display a '➤' icon.
-   **FR-CB-005**: The chatbot MUST use a contrasting emerald green/teal color palette for its UI elements (header, background, message bubbles, input).
-   **FR-CB-006**: The chatbot's color scheme MUST adapt correctly to Docusaurus's light and dark modes.
-   **FR-CB-007**: User message text MUST be dark gray/blue in light mode and light gray/white in dark mode.

### Key Entities

-   **Chatbot UI Components**: FloatingChatButton, ChatWindow, ChatInput, ChatMessage
-   **CSS Variables**: `--chatbot-primary`, `--chatbot-primary-dark`, `--chatbot-primary-light`, `--chatbot-accent`, `--chatbot-bg-light`, `--chatbot-bg-dark`, `--chatbot-text-light`, `--chatbot-text-dark`, `--chatbot-border-color`, `--chatbot-input-bg`, etc.

## Success Criteria

### Measurable Outcomes

-   **SC-CB-001**: Chatbot button accurately positioned and displays correct icons.
-   **SC-CB-002**: Chatbot window opens and closes in the specified bottom-right location.
-   **SC-CB-003**: All chatbot UI elements adhere to the new contrasting emerald green/teal theme in both light and dark modes.
-   **SC-CB-004**: User message text color is updated as specified, ensuring readability.
