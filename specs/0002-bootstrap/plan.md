# Implementation Plan: Docusaurus Textbook Bootstrap & GitHub Pages Deployment

**Branch**: `0002-bootstrap` | **Date**: 2025-12-15 | **Spec**: [specs/0002-bootstrap/spec.md](specs/0002-bootstrap/spec.md)
**Input**: Feature specification from `specs/0002-bootstrap/spec.md` and clarification report from `specs/0002-bootstrap/clarify.md`

## Overview & Goals
This plan outlines the steps to bootstrap the initial Docusaurus v3 project structure for the Physical AI & Humanoid Robotics Textbook. The primary goals are to establish a functional and deploy-ready Docusaurus site within the Spec-Kit Plus `spec-kit/frontend` directory, configure it for automated GitHub Pages deployment, and ensure adherence to the project's Constitution, particularly regarding dark mode and aesthetic. This feature is foundational for all subsequent content creation.

## Summary
The "Docusaurus Textbook Bootstrap" feature will establish the foundational project structure, including Docusaurus v3 in `spec-kit/frontend`, automated GitHub Pages deployment via GitHub Actions, and an initial dark mode aesthetic. This directly supports the project's goal of creating an interactive textbook and prepares the repository for content development.

## Technical Context

**Language/Version**: Node.js v18 LTS (exclusive npm for package management), JavaScript/TypeScript (Docusaurus components)
**Primary Dependencies**: Docusaurus v3 (classic preset), React
**Storage**: N/A (Static site deployment)
**Testing**: Docusaurus local build (`npm run start`), GitHub Actions workflow tests, Lighthouse audits for deployed site
**Target Platform**: Web (GitHub Pages, accessed via modern web browsers)
**Project Type**: Web application (Static Site Generator)
**Performance Goals**: Lighthouse scores of 90 or higher across Performance, Accessibility, Best Practices, and SEO.
**Constraints**:
- Constitution compliance (v1.0.0, e.g., technical accuracy, zero hallucination).
- Dark mode default and professional aesthetic (cool blues/dark grays via CSS variables).
- Spec-Kit Plus folder structure integration (`spec-kit/frontend`, `spec-kit/backend`).
- GitHub Actions for automated deployment to `https://HumaizaNaz.github.io/new-book/`.
- No content beyond boilerplate/placeholders.
- npm as the exclusive package manager.
**Scale/Scope**: Initial project setup; handles static content delivery for a textbook.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The plan strictly adheres to the Physical AI & Humanoid Robotics Textbook Constitution v1.0.0.
- **Interdisciplinary Collaboration**: By establishing a robust platform for content, it supports future collaborative content creation.
- **Ethical AI Development**: The plan ensures a compliant platform for future ethical discussions within the textbook.
- **Robustness & Safety Engineering**: Automated deployment and strict versioning contribute to robust infrastructure.
- **Human-Robot Interaction Design**: The focus on aesthetic and dark mode supports positive user experience.
- **Continuous Learning & Adaptation**: Automated deployment enables rapid iteration and updates.
- **Technical Accuracy & Zero Hallucination**: The plan emphasizes specific versions (Node.js v18 LTS, Docusaurus v3) and precise configuration, preventing technical ambiguity.
- **Co-Learning Pedagogy**: The plan sets up the `sidebars.js` for future curriculum integration, supporting the pedagogical approach.

No violations detected; all gates passed.

## Project Structure

### Documentation (this feature)

```text
specs/0002-bootstrap/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # (N/A for this plan, no specific research needed due to clarified spec)
├── data-model.md        # (N/A for this plan, no data model for static site bootstrap)
├── quickstart.md        # (N/A for this plan, quickstart info integrated into README.md)
├── contracts/           # (N/A for this plan, no API contracts)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
new-book/
├── .github/
│   └── workflows/
│       └── deploy-github-pages.yml
├── .specify/
│   └── memory/
│       └── constitution.md
├── spec-kit/
│   ├── frontend/        # Docusaurus site root
│   │   ├── docs/
│   │   │   └── intro.md
│   │   ├── src/
│   │   │   ├── css/
│   │   │   │   └── custom.css
│   │   │   └── pages/
│   │   │       └── index.js
│   │   ├── static/
│   │   │   └── img/
│   │   ├── docusaurus.config.js
│   │   ├── package.json
│   │   ├── sidebars.js
│   │   └── (other Docusaurus boilerplate)
│   └── backend/         # Agents/Skills (future)
├── docs/                # (Placeholder for future root-level docs if needed, but Docusaurus content is in spec-kit/frontend/docs)
├── src/                 # (Placeholder for future root-level src if needed, but Docusaurus content is in spec-kit/frontend/src)
├── static/              # (Placeholder for future root-level static assets if needed)
├── specs/
│   └── 0002-bootstrap/
│       ├── spec.md
│       ├── checklists/
│       │   └── requirements.md
│       └── plan.md      # This file
├── .gitignore
├── README.md
├── package.json         # (Repository root package.json for project-wide scripts/dependencies)
└── babel.config.js      # (If Docusaurus or other tools require it at root)
```

**Structure Decision**: The project will utilize a hybrid structure. The primary Docusaurus application will reside in `spec-kit/frontend/`, aligning with Spec-Kit Plus. The root directory will contain configuration for GitHub Actions, `README.md`, `.gitignore`, and a root `package.json` for managing project-wide scripts or dependencies, if any.

## Task Breakdown

### Phase 1: Core Setup & Docusaurus Initialization

| Task ID | Priority | Effort | Dependencies | Steps | Responsible |
|:--------|:---------|:-------|:-------------|:------|:------------|
| P1-T01  | P1       | Low    | None         | Create root-level folders: `.github/workflows`, `specs/0002-bootstrap/checklists`, `spec-kit/frontend`, `spec-kit/backend`, `docs`, `src`, `static`. | Agent |
| P1-T02  | P1       | Medium | P1-T01       | Initialize Docusaurus v3 classic preset project inside `spec-kit/frontend/` using `npx create-docusaurus@latest spec-kit/frontend classic`. | Agent |
| P1-T03  | P1       | Low    | P1-T02       | Update `spec-kit/frontend/docusaurus.config.js` for dark mode (`themeConfig.colorMode.defaultMode: 'dark'`) and GitHub Pages deployment details (`url`, `baseUrl`, `projectName`, `organizationName`). | Agent |
| P1-T04  | P1       | Low    | P1-T02       | Update `spec-kit/frontend/sidebars.js` with manual entries for `intro.md` and `index.js`. | Agent |
| P1-T05  | P1       | Low    | P1-T02       | Create `spec-kit/frontend/src/css/custom.css` and define CSS variables for cool blues and dark grays. | Agent |
| P1-T06  | P1       | Low    | P1-T02       | Create `spec-kit/frontend/src/pages/index.js` as the Docusaurus homepage. | Agent |
| P1-T07  | P1       | Low    | P1-T02       | Create `spec-kit/frontend/docs/intro.md` as a placeholder welcome page. | Agent |
| P1-T08  | P1       | Low    | None         | Create/update root `.gitignore` with standard Node.js/Docusaurus exclusions. | Agent |
| P1-T09  | P1       | Medium | P1-T02       | Create `deploy-github-pages.yml` in `.github/workflows/` with the official GitHub Actions workflow for Docusaurus deployment to GitHub Pages. Include steps for `checkout`, `setup-node` (Node.js v18 LTS), `npm install`, `docusaurus build`, `upload-pages-artifact`, and `deploy-pages`. | Agent |
| P1-T10  | P1       | Medium | P1-T09       | Create `README.md` at the repository root with hackathon title, quick start commands (`npm run start` in `spec-kit/frontend/`), and a deploy status badge. | Agent |

### Phase 2: Verification & Initial Build

| Task ID | Priority | Effort | Dependencies | Steps | Responsible |
|:--------|:---------|:-------|:-------------|:------|:------------|
| P2-T01  | P1       | Low    | P1-T01 to P1-T10 | Install root `package.json` dependencies (if any), then navigate to `spec-kit/frontend/` and run `npm install`. | Agent |
| P2-T02  | P1       | Medium | P2-T01       | Run `npm run start` in `spec-kit/frontend/` to verify local Docusaurus build and dark mode default. | Agent |
| P2-T03  | P1       | High   | P1-T10       | Push changes to `main` branch to trigger GitHub Actions workflow and verify successful deployment to GitHub Pages. | Agent/Manual (for push) |
| P2-T04  | P1       | High   | P2-T03       | Verify deployed site at `https://HumaizaNaz.github.io/new-book/` for content, dark mode, and Lighthouse scores (Performance, Accessibility, Best Practices, SEO >= 90). | Agent/Manual |

## Risk Assessment

| Risk                                     | Mitigation                                                                                                                              |
|:-----------------------------------------|:----------------------------------------------------------------------------------------------------------------------------------------|
| GitHub Actions deployment failure        | Implement robust error handling and notifications in the workflow (`deploy-github-pages.yml`). Ensure correct `baseUrl` and `url` configuration. Clear instructions in `README.md`.                                                 |
| Docusaurus build errors                  | Specify exact Node.js version (v18 LTS) and package manager (npm). Provide detailed local build instructions in `README.md`. Use Docusaurus v3.                                 |
| Aesthetic not meeting "professional tech"| Use specific CSS variables for styling. Conduct visual review against design expectations (dark mode, cool blues/dark grays).                                             |
| Inconsistent Spec-Kit Plus structure     | Explicitly create `spec-kit/frontend` and `spec-kit/backend` folders at root. Ensure Docusaurus initializes directly into `spec-kit/frontend`.                           |
| Node.js/npm version conflicts            | Explicitly state Node.js v18 LTS and npm as exclusive package manager in documentation and CI/CD setup.                                                                |
| GitHub Pages configuration               | Ensure GitHub Pages source is configured for "GitHub Actions" in repository settings before deployment.                                                                    |

## Timeline & Milestones (Hackathon Focus)

-   **Day 1 (Initial Hours)**: Complete Phase 1 (Core Setup & Docusaurus Initialization). Target: All files and initial config in place, local build verified.
-   **Day 1 (Mid-day)**: Complete Phase 2 (Verification & Initial Build). Target: Successful GitHub Pages deployment and initial Lighthouse checks.
-   **End of Day 1**: Project is bootstrapped, deployed, and ready for content generation for Module 1.

## Verification & Testing

1.  **Local Build**:
    -   Navigate to `spec-kit/frontend/`.
    -   Run `npm install` and then `npm run start`.
    -   Verify the Docusaurus site loads without errors in a web browser.
    -   Confirm dark mode is the default, and the cool blues/dark grays aesthetic is applied via `custom.css`.
    -   Check navigation for `intro.md` and `index.js`.
2.  **CI/CD Deployment**:
    -   Push a commit to the `main` branch.
    -   Monitor the GitHub Actions workflow for `deploy-github-pages.yml` to ensure it completes successfully.
    -   Verify the deployed site at `https://HumaizaNaz.github.io/new-book/`.
3.  **Lighthouse Audit**:
    -   Run a Lighthouse audit on the deployed site's homepage.
    -   Confirm Performance, Accessibility, Best Practices, and SEO scores are 90 or higher.
4.  **Constitution Compliance**: Manually review all generated files (`docusaurus.config.js`, `custom.css`, `deploy-github-pages.yml`, `README.md`) against the Physical AI & Humanoid Robotics Textbook Constitution v1.0.0 to ensure strict adherence to all principles and technical standards.

## Next Steps
The project is now ready for the next phase.

**Next Recommended Command**: Proceed to `/sp.implement` to execute the tasks outlined in this plan.
Alternatively, the next logical step would be to create/update the `curriculum.yaml` and generate the sidebar for content, as explicitly planned in FR-008.
