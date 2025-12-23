# Tasks for Urdu Translation with Blue Book Icon

**Feature Branch**: `001-urdu-translation`
**Date**: 2025-12-17
**Spec**: /specs/001-urdu-translation/spec.md
**Plan**: /specs/001-urdu-translation/plan.md

## Summary

This document outlines the tasks required to implement the Urdu Translation feature, including creating a Claude agent and skill, developing frontend components for the translation button and modal, integrating translation logic, styling, and testing.

## Task Dependencies

- **Phase 1 (Setup)** must be completed before any other phases.
- **Phase 2 (Foundational)** must be completed before **Phase 3**.
- **Phase 3 (User Story 1)** must be completed before **Phase 4** and **Phase 5**.
- **Phase 4 (Polish & Cross-Cutting Concerns)** can be done in parallel with testing, but depends on completion of Phase 3.
- **Phase 5 (Testing)** can be done in parallel with polishing, but depends on completion of Phase 3.

## Parallel Execution Examples

- Within Phase 3, some frontend component tasks might be developed in parallel, but their integration will be sequential.
- Within Phase 4, styling tasks for different components can be parallelized.

## Implementation Strategy

The implementation will follow an iterative approach, prioritizing the core translation functionality and then refining the UI and integration. The MVP will focus on a functional translation button and modal, with subsequent polish and optional features.

## Phases

### Phase 1: Setup

Goal: Ensure necessary directories and basic files are in place.

- [X] T001 Create directory `backend/.claude/agents/urdu-translator/`
- [X] T002 Create directory `backend/.claude/skills/urdu-translator/`
- [X] T003 Create directory `frontend/src/components/` if it doesn't exist
- [X] T004 Create directory `frontend/src/css/` if it doesn't exist

### Phase 2: Foundational

Goal: Implement the core Claude agent and skill for Urdu translation.

- [X] T005 Implement `backend/.claude/agents/urdu-translator/AGENT.md` with Roman Urdu greeting and translation capabilities
- [X] T006 Implement `backend/.claude/skills/urdu-translator/SKILL.md` to handle translation requests, preserving technical terms and code blocks

### Phase 3: User Story 1 - Translate Chapter Content [US1]

Goal: Allow users to translate chapter content to Urdu and view it in a modal.

- [X] T007 [US1] Create `frontend/src/components/UrduTranslateButton.jsx` with text "اردو میں ترجمہ کریں" and a blue book icon (📘 or SVG)
- [X] T008 [US1] Create `frontend/src/components/UrduTranslationModal.jsx` to display translated markdown content in a scrollable format
- [X] T009 [US1] Implement chapter text extraction logic within `frontend/src/components/UrduTranslateButton.jsx` or a utility, to get clean text from the current Docusaurus page
- [X] T010 [US1] Implement translation request logic in `frontend/src/components/UrduTranslateButton.jsx` to send extracted text to the backend's `/chat` endpoint and receive translated content

### Phase 4: Polish & Cross-Cutting Concerns

Goal: Enhance UI, integrate button globally, and consider optional agent dropdown integration.

- [X] T011 [P] Style `frontend/src/components/UrduTranslateButton.jsx` and `frontend/src/components/UrduTranslationModal.jsx` using `frontend/src/css/UrduTranslate.css` (blue gradient, dark mode, responsive)
- [X] T012 Inject `UrduTranslateButton` globally into the Docusaurus theme layout (e.g., `frontend/src/theme/Root.js` or similar page header file)
- [ ] T013 Optional: Integrate Urdu Translator into an existing agent dropdown mechanism

### Phase 5: Testing

Goal: Verify the functionality, accuracy, and UI of the Urdu translation feature.

- [X] T014 Test translation across at least three different chapter pages to verify accurate Roman Urdu translation, preservation of code blocks, and technical terms.
- [X] T015 Verify UI responsiveness, blue theme adherence, and dark mode compatibility for the button and modal.
