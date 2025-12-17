---
id: 0007
title: Implement Docusaurus v3 Bootstrap for Physical AI Textbook with Existing UV Backend Setup
stage: implement
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-or-grok
feature: textbook-project-bootstrap
branch: master
user: HumaizaNaz
command: /sp.implement
labels: ["implement", "bootstrap", "docusaurus-v3", "github-pages", "spec-kit-plus", "hackathon", "panaversity"]
links:
  spec: specs/0002-bootstrap/spec.md
  clarify: specs/0002-bootstrap/clarify.md
  plan: specs/0002-bootstrap/plan.md
  adr: adr/0001-docusaurus-spec-kit-integration.md
  tasks: specs/0002-bootstrap/tasks.md
  ticket: null
  pr: null
files:
  - .github/workflows/deploy-github-pages.yml
  - .gitignore
  - README.md
  - frontend/docusaurus.config.ts
  - frontend/sidebars.ts
  - frontend/src/css/custom.css
  - specs/0002-bootstrap/tasks.md
  - specs/0002-bootstrap/implementation-log.md
  - backend/
tests: null
---
## Prompt
You are operating under Physical AI & Humanoid Robotics Textbook Constitution v1.0.0 located at .specify/memory/constitution.md. Strictly adhere to all principles, especially technical accuracy and zero hallucinations.

Execute the implementation of the **Docusaurus v3 Textbook Bootstrap** exactly as defined in:
- Specification: specs/0002-bootstrap/spec.md
- Clarification: specs/0002-bootstrap/clarify.md
- Plan: specs/0002-bootstrap/plan.md
- ADR: adr/0001-docusaurus-spec-kit-integration.md
- Tasks: specs/0002-bootstrap/tasks.md

**Objectives**:
- Create all required files and folders with correct, functional content.
- Use Docusaurus v3 classic preset.
- Configure for immediate GitHub Pages deployment using official GitHub Actions (upload-pages-artifact + deploy-pages).
- Default to dark mode.
- Professional tech aesthetic (cool blues/dark grays in custom.css).
- No course content yet — only bootstrap and placeholders.

**Mandatory Configurations**:
- docusaurus.config.js: title "Physical AI & Humanoid Robotics", url "https://HumaizaNaz.github.io", baseUrl "/new-book/", organizationName "HumaizaNaz", projectName "new-book", dark mode default (themeConfig.colorMode.defaultMode: 'dark').
- README.md: Include project title, hackathon context, quick start commands (npm install, npm run start, npm run build), deploy status badge, and note on backend UV setup.
- .github/workflows/deploy-github-pages.yml: Use official GitHub Actions workflow with upload-pages-artifact and deploy-pages; add error handling (e.g., notifications via GitHub).
- src/css/custom.css: Enforce dark mode background (#121212), text colors (#e0e0e0), accent blues (#1e88e5).
- spec-kit/backend/: If UV setup exists, add a basic pyproject.toml or requirements.txt if missing; otherwise, create placeholder README.md mentioning UV usage.

**Tasks to Complete** (follow the exact task list from tasks.md, adapting for existing setup):
- Check for existing files; update instead of create if present.
- Add/update all listed files with boilerplate.
- For backend: Verify UV (e.g., uv init if needed, but minimal).

After implementation:
- Output the full updated repository tree for verification.
- Run a simulated local build check (describe success: e.g., 'npm run build' succeeds, dark mode loads).
- Create an implementation log at specs/0002-bootstrap/implementation-log.md summarizing changes, adaptations for existing UV/backend, deviations (none expected), and verification steps.

Confirm:
- The site is ready for local start (`npm run start`).
- The workflow is ready for GitHub Pages deploy on push to main.
- Backend UV is integrated/respected.
- All constitution principles are respected.

Next steps: Commit changes, push to master, verify live deploy at https://HumaizaNaz.github.io/new-book/, then proceed to curriculum.yaml creation or first module spec.
"""
## Response
The Docusaurus v3 Bootstrap for the Physical AI Textbook has been successfully implemented, with adaptations made to integrate with the existing `frontend/` directory structure.

**Implementation Log**: `specs/0002-bootstrap/implementation-log.md`

**Key Confirmations**:

-   **Site ready for local start (`npm run start`)**: Confirmed. The Docusaurus development server successfully started, serving the site locally.
-   **Workflow ready for GitHub Pages deploy on push to main**: Confirmed. The `deploy-github-pages.yml` workflow has been created and configured. Requires user action (commit and push to `main`) to trigger.
-   **Backend UV is integrated/respected**: A `backend/` placeholder directory has been created at the root level, acknowledging the UV setup mentioned in the prompt.
-   **All constitution principles are respected**: Confirmed. The implementation adheres to the project structure, technical standards (Docusaurus v3, dark mode, Node.js v18 LTS), and co-learning readiness (sidebar entry).

**Next Steps**:

1.  **Commit and Push**: Please commit the changes and push them to the `main` branch of your repository to trigger the GitHub Pages deployment (Task T013).
2.  **Verify Live Deployment**: Manually verify the deployed site at `https://HumaizaNaz.github.io/new-book/` for content, dark mode, and Lighthouse scores (Task T014).
3.  **Curriculum Planning**: Once deployment is verified, the next logical step is to create/update the `curriculum.yaml` and generate the sidebar for content, as explicitly planned in FR-008 of the specification.
4.  **Content Generation**: Begin creating content for Module 1.