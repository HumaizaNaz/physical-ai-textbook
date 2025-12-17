# ADR-0001: Use Docusaurus v3 with Spec-Kit Plus for Textbook Frontend

## Status
Accepted

## Date
2025-12-15

## Context
We are bootstrapping an interactive AI-native textbook titled "Physical AI & Humanoid Robotics" using Docusaurus v3 as the frontend, deployed to GitHub Pages. The project follows Spec-Kit Plus patterns with `spec-kit/frontend` and `spec-kit/backend` folders. The constitution requires strict spec-driven development, technical accuracy, dark mode default, and preparation for co-learning elements. The primary goal is to establish a functional and deploy-ready Docusaurus site within the Spec-Kit Plus structure.

## Decision
Adopt **Docusaurus v3 classic preset** with Spec-Kit Plus integration and GitHub Pages deployment for the textbook frontend.

## Consequences
-   **Positive**:
    -   Excellent documentation features (versioning, i18n-ready, plugins for interactivity), built-in dark mode support, fast static site generation, and seamless integration with GitHub Pages.
    -   Aligns perfectly with Spec-Kit Plus workflow, where specifications and related documentation live alongside the codebase, promoting spec-driven development.
    -   Strong community support and extensive plugin ecosystem.
-   **Negative**:
    -   Slightly larger bundle size compared to more minimal static site generators.
    -   Requires a Node.js environment for local development and build processes (which is acceptable for the hackathon and future contributors).
-   **Risks**:
    -   Potential initial learning curve for contributors unfamiliar with Docusaurus or Spec-Kit Plus.
    -   Dependency management for Docusaurus and its plugins needs careful attention to avoid conflicts or outdated packages.

## Alternatives Considered
1.  **Docusaurus v3 classic preset (Chosen)**: Offers a highly optimized solution for documentation sites with built-in features aligning with project needs. Quick setup for hackathon, robust for future expansion.
2.  **Next.js with MDX + custom static generation**: While powerful, Next.js would introduce more complexity for a documentation-focused site, requiring more custom development for features like search, versioning, and sidebars, which are out-of-the-box in Docusaurus.
3.  **Pure Markdown + Hugo or MkDocs**: These are lightweight options but lack the rich feature set (e.g., interactive components, plugin ecosystem for co-learning elements) and the integrated development experience that Docusaurus provides, which is critical for an interactive AI-native textbook.

## References
-   **Spec**: [specs/0002-bootstrap/spec.md](specs/0002-bootstrap/spec.md)
-   **Clarification Report**: [specs/0002-bootstrap/clarify.md](specs/0002-bootstrap/clarify.md)
-   **Plan**: [specs/0002-bootstrap/plan.md](specs/0002-bootstrap/plan.md)
-   **Repository**: https://github.com/HumaizaNaz/new-book