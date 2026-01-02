# Constitution for Homepage Features Refactor

## Preamble & Feature Overview
This document establishes the foundational principles for the **Homepage Features Refactor**. All development, specifications, and tasks related to this feature MUST adhere to this constitution.

**Project Name**: Homepage Features Refactor

## Non-Negotiable Principles

### 1. Component-Based Architecture
- **Modularity**: The homepage features section **MUST** be built using a modular, component-based architecture.
- **Decomposition**: The main `HomepageFeatures` component **MUST** be decomposed into smaller, reusable sub-components for each distinct section (e.g., Stats, Modules, CTA).

### 2. Code Organization
- **Location**: All refactored components **MUST** reside within the `frontend/src/components/HomepageFeatures/` directory.
- **Styling**: Component-specific styles **MUST** be co-located with their respective components using CSS Modules (`styles.module.css`).

### 3. Theming and Design
- **Consistency**: The new components **MUST** adhere to the overall design language of the website, using the Docusaurus Infima CSS framework and its variables.
- **Glassmorphism**: A "glassmorphism" effect **MUST** be implemented for card-like elements, with variables defined in the global `frontend/src/css/custom.css` file to ensure consistency across light and dark modes.

### 4. Non-Regression
- **Existing Functionality**: The refactoring **MUST NOT** break or alter the existing functionality or appearance of the homepage.
- **Responsiveness**: The homepage **MUST** remain fully responsive across all standard screen sizes.

### 5. Technology Stack
- **Frontend**: The components **MUST** be built using React and TypeScript, consistent with the Docusaurus framework.
- **Dependencies**: The use of `react-icons` for iconography is approved and **MUST** be managed as a project dependency.

## Enforcement Clause
This constitution is the **single source of truth** for this feature. All future agent prompts, content generation, code, and contributions related to this feature MUST obey these principles 100%. No deviations are allowed.

## Governance
This Constitution is the governing document for the specified feature. Amendments require a formal proposal and review.
