# Tasks: Docusaurus Textbook Bootstrap & GitHub Pages Deployment

**Input**: Design documents from `specs/0002-bootstrap/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Phase 1: Core Setup & Docusaurus Initialization

### User Story 1 - Docusaurus Site Initialization (P1) 🎯 MVP

**Goal**: Establish the core Docusaurus site within `spec-kit/frontend` and prepare it for content.

**Independent Test**: The existence of a complete Docusaurus project structure (including `package.json`, `docusaurus.config.js`, `src/`, `docs/`, `static/` folders) within `spec-kit/frontend/` confirms successful initialization.

- [X] T001 [P] [US1] Create root-level folders: `.github/workflows`, `specs/0002-bootstrap/checklists`, `spec-kit/frontend` (ADAPTED: using existing `frontend/` for Docusaurus), `spec-kit/backend` (ADAPTED: creating `backend/` at root for UV config), `docs` (ADAPTED: using `frontend/docs`), `src` (ADAPTED: using `frontend/src`), `static` (ADAPTED: using `frontend/static`).
- [X] T002 [US1] Initialize Docusaurus v3 classic preset project inside `spec-kit/frontend/` (ADAPTED: confirmed existing Docusaurus v3 classic preset in `frontend/`).
- [X] T003 [US1] Update `spec-kit/frontend/docusaurus.config.js` (ADAPTED: `frontend/docusaurus.config.ts`) for dark mode (`themeConfig.colorMode.defaultMode: 'dark'`) and GitHub Pages deployment details (`url`, `baseUrl`, `projectName`, `organizationName`).
- [X] T004 [US1] Update `spec-kit/frontend/sidebars.js` (ADAPTED: `frontend/sidebars.ts`) with manual entries for `intro.md`.
- [X] T005 [P] [US1] Create `spec-kit/frontend/src/css/custom.css` (ADAPTED: updated `frontend/src/css/custom.css`) and define CSS variables for cool blues and dark grays.
- [X] T006 [P] [US1] Create `spec-kit/frontend/src/pages/index.js` (ADAPTED: confirmed existing `frontend/src/pages/index.tsx` as Docusaurus homepage).
- [X] T007 [P] [US1] Create `spec-kit/frontend/docs/intro.md` (ADAPTED: confirmed existing `frontend/docs/intro.md`) as a placeholder welcome page.
- [X] T008 [P] [US1] Create/update root `.gitignore` with standard Node.js/Docusaurus exclusions (`.docusaurus`, `node_modules`, `build`, etc.).

### User Story 2 - Automated GitHub Pages Deployment Setup (P1)

**Goal**: Configure automated deployment to GitHub Pages via GitHub Actions.

**Independent Test**: A successful run of the GitHub Actions workflow for deployment to GitHub Pages, resulting in a publicly accessible site at `https://HumaizaNaz.github.io/new-book/`.

- [X] T009 [US2] Create `deploy-github-pages.yml` in `.github/workflows/` with the official GitHub Actions workflow for Docusaurus deployment to GitHub Pages. Include steps for `actions/checkout`, `actions/setup-node` (Node.js v18 LTS), `npm install`, `docusaurus build`, `actions/upload-pages-artifact`, and `actions/deploy-pages`.
- [X] T010 [US2] Create `README.md` at the repository root with hackathon title, quick start commands (`npm run start` in `frontend/`), and a deploy status badge.

## Phase 2: Verification & Initial Build

### User Story 1, 2, 3 - Verification (P1)

**Goal**: Verify local build, aesthetic, and successful deployment to GitHub Pages.

**Independent Test**: Successful execution of `npm run start` locally, and a successful GitHub Pages deployment with Lighthouse scores >=90 and dark mode default.

- [X] T011 [P] [US1, US2, US3] Install root `package.json` dependencies (if any), then navigate to `frontend/` and run `npm install`.
- [X] T012 [P] [US1, US3] Run `npm run start` in `frontend/` to verify local Docusaurus build and dark mode default, and aesthetic.
- [ ] T013 [US2, US3] Push changes to `main` branch to trigger GitHub Actions workflow and verify successful deployment to GitHub Pages at `https://HumaizaNaz.github.io/new-book/`.
- [ ] T014 [P] [US2, US3] Verify deployed site at `https://HumaizaNaz.github.io/new-book/` for content, dark mode, and Lighthouse scores (Performance, Accessibility, Best Practices, SEO >= 90).

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Phase 1 (Core Setup & Docusaurus Initialization)**: No dependencies - can start immediately.
-   **Phase 2 (Verification & Initial Build)**: Depends on Phase 1 completion.

### Within Each User Story

-   Tasks `T002` through `T007` for User Story 1 depend on `T001`.
-   Task `T009` for User Story 2 depends on `T002`.
-   Task `T010` for User Story 2 depends on `T009`.
-   Task `T011` depends on all Phase 1 tasks.
-   Task `T012` depends on `T011`.
-   Task `T013` depends on `T010` and successful local build/verification.
-   Task `T014` depends on `T013`.

### Parallel Opportunities

-   Tasks `T001` and `T008` can run in parallel.
-   Tasks `T005`, `T006`, `T007` can run in parallel after `T002` and `T004` are complete.
-   Tasks `T011`, `T012`, `T014` involve verification and can have parallel components (e.g., local vs. deployed checks).
