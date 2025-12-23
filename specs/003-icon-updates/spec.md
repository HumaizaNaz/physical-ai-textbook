# Feature Specification: Icon Updates for Chatbot and Urdu Translation Button

**Feature Branch**: `001-urdu-translation` (part of polish for this branch)
**Created**: 2025-12-21
**Status**: Completed

## Summary

This specification documents the replacement of emoji icons with SVG icons for both the floating RAG chatbot button and the Urdu translation button. The goal is to enhance visual quality, professionalism, and theme consistency across the application.

## User Scenarios & Testing

### User Story 1 - Chatbot Floating Button Icon Update (Priority: P1)

**Goal**: The chatbot's floating button should use a professional SVG icon that clearly represents chat functionality.

**Independent Test**: The chatbot button is visible, and it displays a clean, scalable SVG chat icon instead of an emoji.

**Acceptance Scenarios**:

1.  **Given** the user is on any page, **When** the page loads, **Then** a floating chat button is visible at the bottom-right displaying a new SVG chat icon (when closed).
2.  **Given** the floating chat button is visible, **When** the Docusaurus theme is switched between light and dark modes, **Then** the SVG chat icon's color correctly adapts (light mode: `#1e293b`, dark mode: `white`).

### User Story 2 - Urdu Translation Button Icon Update (Priority: P1)

**Goal**: The Urdu translation button should use a professional SVG icon representing a book or translation, visible when not hovered, and adapt its color to the theme.

**Independent Test**: The Urdu translation button is visible, displays a clean, scalable SVG book icon, and the text only appears on hover.

**Acceptance Scenarios**:

1.  **Given** the user is on any chapter page, **When** the page loads, **Then** an Urdu translation button is visible at the bottom-left displaying a new SVG open book icon.
2.  **Given** the Urdu translation button is visible, **When** the Docusaurus theme is switched between light and dark modes, **Then** the SVG book icon's color correctly adapts (white).
3.  **Given** the Urdu translation button is visible, **When** the user hovers over it, **Then** the text "اردو میں ترجمہ کریں" appears, and the button expands.

## Requirements

### Functional Requirements

-   **FR-ICN-001**: The chatbot's floating button MUST use an SVG icon for its closed state, replacing the emoji.
-   **FR-ICN-002**: The chatbot's SVG icon MUST inherit its color from the theme-dependent `--chatbot-icon-color` CSS variable.
-   **FR-ICN-003**: The Urdu translation button MUST use an SVG icon (open book), replacing the emoji.
-   **FR-ICN-004**: The Urdu translation button's SVG icon MUST inherit its color (white) and scale correctly.
-   **FR-ICN-005**: The Urdu translation button's text ("اردو میں ترجمہ کریں") MUST only be visible on hover.

### Key Entities

-   **Components**: FloatingChatButton.tsx, UrduTranslateButton.tsx
-   **CSS Files**: FloatingChatButton.module.css, UrduTranslate.css
-   **Icons**: SVG Chat Icon, SVG Book Icon

## Success Criteria

### Measurable Outcomes

-   **SC-ICN-001**: Chatbot button displays SVG chat icon in all themes.
-   **SC-ICN-002**: Urdu translation button displays SVG book icon, and text appears only on hover.
-   **SC-ICN-003**: Both SVG icons are correctly styled and themed.
