# Docusaurus Platform Specification

## Context
This document specifies the core usage and configuration of Docusaurus as the frontend platform for the "physical-ai-textbook" project. It outlines how Docusaurus features are leveraged and customized to meet the project's requirements.

## Core Principles
*   **Content-Centric:** Docusaurus is primarily used for structured documentation and textbook content delivery.
*   **Mobile-First Responsive Design:** The platform must render effectively across various device sizes.
*   **Dark Mode Default:** Adherence to the project's locked decision for dark mode as the default theme.
*   **Extensibility:** Utilize Docusaurus plugin and theming capabilities for custom features.

## Key Configurations
### Theming
*   **Custom CSS (`frontend/src/css/custom.css`):**
    *   Imports `Roboto Slab` font for headings and navigation.
    *   Defines project-specific color palettes for both light and dark modes (violet/purple for light, magenta/mulberry for dark).
    *   Customizes navbar and footer background and text colors.
*   **Component-Specific Styling (`.module.css` files):**
    *   Leverages CSS Modules for localized styling of React components (e.g., `HomepageFeatures`).

### Navigation
*   **Sidebar Configuration (`sidebars.ts`):** Defines the hierarchical structure of the textbook chapters and modules.
*   **Navbar Configuration (`docusaurus.config.ts`):** Configures primary navigation links and branding ("physical-ai-textbook").

### Content Authoring
*   **Markdown/MDX:** Primary format for writing textbook content, allowing for rich media and interactive components.
*   **Code Blocks:** Standard Docusaurus code block functionality for displaying code snippets.

## Custom Components
*   **Co-Learning Elements:** Integration points for interactive components specific to the learning experience (details specified in `co-learning-element-spec.md`).
*   **Live Curl Integration:** Mechanism for displaying and interacting with live `curl` commands linked to the FastAPI backend (details specified in `live-curl-integration-spec.md`).

## Integration Points
*   **FastAPI Backend:** The Docusaurus frontend will make requests to the FastAPI backend for interactive elements, grading, and live demo data.

## Versioning
*   Standard Docusaurus documentation versioning capabilities will be utilized if multiple versions of the textbook are required.

## References
*   `docusaurus.config.ts`
*   `sidebars.ts`
*   `frontend/src/css/custom.css`
*   `00-ADR.md`
*   `00-PLAN.md`
