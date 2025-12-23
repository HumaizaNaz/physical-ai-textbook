# Tasks for Chatbot UI Polish

**Feature Branch**: `001-urdu-translation` (part of polish for this branch)
**Date**: 2025-12-21
**Spec**: /specs/002-chatbot-ui-polish/spec.md
**Plan**: /specs/002-chatbot-ui-polish/plan.md

## Summary

This document outlines the tasks performed to implement the UI/UX polish for the RAG chatbot, including positioning, icon changes, and applying a contrasting, theme-adaptive color palette. All tasks are marked as completed.

## Task Dependencies

All tasks within this feature are conceptually grouped under 'Polish & Cross-Cutting Concerns' and have been implemented iteratively.

## Implementation Strategy

The implementation followed an iterative approach, applying CSS and component modifications based on user feedback to achieve the desired aesthetic and functional improvements.

## Phases

### Phase 1: Chatbot Positioning and Iconography

Goal: Correctly position the chatbot and update its icons.

- [X] T001 Update `frontend/src/components/ChatBot/FloatingChatButton.tsx` to change the main icon from '🤖' to '🗨️'.
- [X] T002 Update `frontend/src/components/ChatBot/ChatWindow.module.css` to remove the fullscreen overlay and position the chat window in the bottom-right corner.
- [X] T003 Update `frontend/src/components/ChatBot/ChatWindow.tsx` to remove the `styles.overlay` div.
- [X] T004 Update `frontend/src/components/ChatBot/ChatInput.tsx` to change the send button icon from '🤖' to '➤'.

### Phase 2: Chatbot Theming and Readability

Goal: Apply a contrasting, theme-adaptive color palette and ensure readability.

- [X] T005 Update `frontend/src/components/ChatBot/FloatingChatButton.module.css` to use custom `--chatbot-` CSS variables for the button's background gradient and ensure the icon color is `#1e293b` in light mode and `white` in dark mode.
- [X] T006 Update `frontend/src/components/ChatBot/ChatWindow.module.css` to define and apply custom `--chatbot-` CSS variables for the chat window's header, background, and text, using an emerald green/teal palette.
- [X] T007 Update `frontend/src/components/ChatBot/ChatInput.module.css` to use custom `--chatbot-` CSS variables for the input container, input field, and send button, aligning with the new theme.
- [X] T008 Update `frontend/src/components/ChatBot/ChatMessage.module.css` to use custom `--chatbot-` CSS variables for message bubbles (user and assistant) and source chips, ensuring a consistent and contrasting theme.
- [X] T009 Update `frontend/src/components/ChatBot/ChatMessage.module.css` to set the user message text color to `var(--chatbot-text-light)` (dark gray/blue) in light mode and `var(--chatbot-text-dark)` (light gray/white) in dark mode.

## Summary of Completed Work

All tasks outlined for the Chatbot UI Polish feature have been successfully implemented and verified through iterative user feedback. The chatbot now features:
-   Correct positioning (bottom-right).
-   Appropriate chat icons (`🗨️` for main, `➤` for send).
-   A contrasting emerald green/teal theme that adapts to Docusaurus's light and dark modes.
-   Improved readability with updated user message text color.