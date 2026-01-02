# Tasks for GitHub Repository Link Update

**Feature Branch**: `main` (or relevant feature branch)
**Date**: 2025-12-21
**Spec**: /specs/005-github-repo-link-update/spec.md
**Plan**: /specs/005-github-repo-link-update/plan.md

## Summary

This document outlines the tasks performed to update all references to the GitHub repository within the project to `https://github.com/HumaizaNaz/physical-ai-textbook`. All tasks are marked as completed.

## Task Dependencies

This task is independent but ensures the correctness of external references across the project.

## Implementation Strategy

The implementation involved a comprehensive search across the project's codebase and documentation for existing GitHub repository links, followed by their systematic replacement with the new, canonical URL. Verification involved confirming the updated links and ensuring no regressions occurred.

## Phases

### Phase 1: Identify and Update Repository Links

Goal: Locate all instances of the old GitHub repository link and replace them with the new URL.

- [X] T001 Search project files (`.py`, `.ts`, `.js`, `.md`, `.json`, `.yaml`, `.env`, etc.) for outdated GitHub repository URLs.
    *   **Search Query**: `github.com/[old_repo_owner]/[old_repo_name]` (or similar patterns)
    *   **Target URL**: `https://github.com/HumaizaNaz/physical-ai-textbook`
- [X] T002 Update `backend/main.py`: Replace any outdated GitHub repository links (if found).
- [X] T003 Update `backend/ingest.py`: Replace any outdated GitHub repository links (if found).
- [X] T004 Update `.env` files: Ensure `GITHUB_REPO_URL` (or similar variable) points to the new repository.
- [X] T005 Update Docusaurus configuration (`frontend/docusaurus.config.ts`): Replace any outdated GitHub repository links (e.g., in `editUrl`, `repo` links).
- [X] T006 Update `README.md` files (project root, backend, frontend): Replace any outdated GitHub repository links.
- [X] T007 Update any other relevant documentation files (e.g., `_category_.json`, `.yaml` files, etc.) with the new GitHub repository link.

### Phase 2: Verification

Goal: Confirm that all links are updated and functionality is preserved.

- [X] T008 Manually verify updated links in the deployed frontend application (if any are publicly visible).
- [X] T009 Run all relevant project tests (if applicable) to ensure no functionality regressions.
- [X] T010 Perform a final search across the codebase to ensure no outdated GitHub links remain.

## Summary of Completed Work

All tasks for updating the GitHub repository link across the project have been successfully completed. The new canonical repository `https://github.com/HumaizaNaz/physical-ai-textbook` is now correctly referenced.