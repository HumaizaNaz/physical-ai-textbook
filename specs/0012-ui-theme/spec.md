# Feature Specification: UI Theme and Dark/Light Mode Toggle Enhancement

**Feature ID**: 0012
**Branch**: 0012-ui-theme
**Status**: Ready for Implementation

## Color Palette

### Dark Mode (Default)
-   **Background**: `#0f172a` (Deep Blue/Gray)
-   **Cards/Surfaces**: `#1e293b` (Slightly Lighter Blue/Gray)
-   **Text**: `#e2e8f0` (Off-white/Light Gray)
-   **Accent**: `#0ea5e9` (Vibrant Blue)

### Light Mode
-   **Background**: `#f8fafc` (Very Light Gray)
-   **Cards/Surfaces**: `#ffffff` (White)
-   **Text**: `#1e293b` (Dark Blue/Gray)
-   **Accent**: `#0284c7` (Medium Blue)

## Toggle Button Design Spec

-   **Shape**: Pill-shaped toggle button.
-   **Glow Effect**: Subtle glow around the button, especially when active or hovered.
-   **Icons**:
    -   **Light Mode**: Sun icon.
    -   **Dark Mode**: Moon icon with an orbiting ring animation around it.
-   **Transition**: Smooth visual transition when switching between dark and light mode (e.g., fade, subtle scale).

## Navbar Updates

-   **Site Title**: Update the navbar title to "Physical AI & Humanoid Robotics Textbook".
-   **"Tutorial" Link**: Remove the "Tutorial" link from the navbar if present.

## User Stories

### User Story 1 - Engaging Visual Experience (Priority: P1)

As a user, I want the textbook to have a modern, futuristic robotics theme, so that the visual experience is engaging, professional, and reflects the advanced nature of the content.

### User Story 2 - Seamless Theme Switching (Priority: P1)

As a user, I want a smooth dark/light mode toggle button with intuitive icons and animations, so that I can easily switch themes based on my preference without any visual glitches or abrupt changes.

### User Story 3 - Consistent Default Theme (Priority: P1)

As a user, I want the dark mode to be the default theme on initial load, so that the primary visual experience aligns with the established aesthetic of a modern, tech-focused textbook.

## Functional Requirements

-   **FR-001**: The Docusaurus `navbar.title` MUST be updated to "Physical AI & Humanoid Robotics Textbook".
-   **FR-002**: The Docusaurus navbar MUST NOT display a link to "Tutorial" if present.
-   **FR-003**: The site MUST implement a dark mode theme using the specified color palette: Background `#0f172a`, cards `#1e293b`, text `#e2e8f0`, accent `#0ea5e9`.
-   **FR-004**: The site MUST implement a light mode theme using the specified color palette: Background `#f8fafc`, cards `#ffffff`, text `#1e293b`, accent `#0284c7`.
-   **FR-005**: The site MUST feature a toggle button for switching between dark and light mode.
-   **FR-006**: The toggle button MUST have a pill shape with a subtle glow effect.
-   **FR-007**: The toggle button MUST display a sun icon for light mode and a moon icon with an orbiting ring animation for dark mode.
-   **FR-008**: The transition between dark and light mode MUST be smooth.

## Non-Functional Requirements

-   **NFR-001**: The overall UI theme MUST convey a professional, balanced, and "wow" factor, visually inspired by NVIDIA/Figure.ai/Tesla AI sites.
-   **NFR-002**: The dark mode MUST be the default theme on initial load.
-   **NFR-003**: There MUST be no visual glitches or abrupt changes during theme switching.
-   **NFR-004**: The theme changes MUST be consistently applied across all Docusaurus pages.
-   **NFR-005**: The implementation MUST adhere to the Physical AI & Humanoid Robotics Textbook Constitution v1.0.0.

## Success Criteria (mandatory - measurable)

-   **SC-001**: The Docusaurus navbar displays "Physical AI & Humanoid Robotics Textbook" as its title and does not contain any "Tutorial" link.
-   **SC-002**: The dark mode theme with the specified color palette (`#0f172a`, `#1e293b`, `#e2e8f0`, `#0ea5e9`) is consistently applied across the site.
-   **SC-003**: The light mode theme with the specified color palette (`#f8fafc`, `#ffffff`, `#1e293b`, `#0284c7`) is consistently applied across the site when activated via the toggle.
-   **SC-004**: The dark/light mode toggle button is pill-shaped, displays the correct sun/moon icons with the orbiting ring animation for dark mode, and provides a smooth visual transition upon activation.
-   **SC-005**: The site's aesthetic is perceived by stakeholders as modern, futuristic, professional, and visually appealing, consistent with the inspirations.
-   **SC-006**: All links and interactive elements function correctly in both dark and light modes without visual glitches.
