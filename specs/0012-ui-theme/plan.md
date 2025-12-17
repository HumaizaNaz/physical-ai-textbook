# Implementation Plan: UI Theme and Dark/Light Mode Toggle Enhancement

**Branch**: `0012-ui-theme` | **Date**: 2025-12-16 | **Spec**: [specs/0012-ui-theme/spec.md](specs/0012-ui-theme/spec.md)
**Input**: Feature specification from `specs/0012-ui-theme/spec.md`

## Overview & Goals
This plan outlines the steps to implement a modern, futuristic robotics UI theme for the Docusaurus site, focusing on a refined dark mode experience and a custom, animated dark/light mode toggle. The primary goals are to enhance the visual appeal, improve user experience through intuitive theming, and ensure compliance with the Constitution's aesthetic and dark mode default requirements.

## Summary
The "UI Theme and Dark/Light Mode Toggle Enhancement" feature will overhaul the visual design of the Docusaurus textbook, applying specified color palettes for dark and light modes. It will introduce a custom-designed, animated theme toggle button and update the navbar title. This ensures a professional, engaging, and consistent aesthetic inspired by leading AI/robotics companies.

## Technical Context

**Language/Version**: Node.js v18 LTS (npm), JavaScript/TypeScript (Docusaurus, React).
**Primary Dependencies**: Docusaurus v3, React.
**Storage**: N/A (UI configuration and assets).
**Testing**: Local Docusaurus build, visual inspection across components, browser compatibility check, Lighthouse audits (for accessibility).
**Target Platform**: Web (GitHub Pages, Docusaurus static site).
**Project Type**: Web application (Static Site Generator - UI Enhancement).
**Performance Goals**: Smooth UI transitions, no noticeable lag during theme switching (<100ms), Lighthouse scores >= 90 for Performance and Accessibility.
**Constraints**:
- Constitution compliance (v1.0.0, especially dark mode default and professional aesthetic).
- Specified color palettes for dark and light modes.
- Custom toggle design (pill shape, glow, sun/moon icons with orbiting ring animation).
- Smooth transition between modes.
- Compatibility with existing Docusaurus structure and content.
**Scale/Scope**: Site-wide UI theming, affecting all Docusaurus pages.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The plan strictly adheres to the Physical AI & Humanoid Robotics Textbook Constitution v1.0.0.
-   **Docusaurus Compliance**: This feature directly enhances the "Dark Mode Default" and "Professional aesthetic" requirements.
-   **Technical Accuracy**: Implementation will involve standard Docusaurus theming practices and React component development.
-   **Human-Robot Interaction Design**: A polished and intuitive UI directly contributes to a better user experience, aligning with the spirit of good interaction design.

No violations detected; all gates passed.

## Project Structure

### Documentation (this feature)

```text
specs/0012-ui-theme/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── checklists/
│   └── requirements.md  # Spec quality checklist
└── (other artifacts as generated)
```

### Source Code (repository root, specifically Docusaurus content in `frontend/`)

```text
frontend/
├── src/css/custom.css        # Main stylesheet for global theme variables
├── src/components/ThemeToggle/ # New directory for custom toggle component
│   ├── index.tsx
│   ├── styles.module.css
│   └── assets/                  # Icons for sun/moon (SVG/PNG)
├── docusaurus.config.ts      # For navbar title update and default color mode
```

## Phase Breakdown

### Phase 1: Docusaurus Configuration & Navbar Updates (Estimated: 30 min)

**Goal**: Configure Docusaurus for the new default dark mode and update the navbar title.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P1-T01  | Low    | None         | `frontend/docusaurus.config.ts` | `docusaurus.config.ts` has `navbar.title` set to "Physical AI & Humanoid Robotics Textbook". |
| P1-T02  | Low    | None         | `frontend/docusaurus.config.ts` | `docusaurus.config.ts` `navbar.items` no longer contains "Tutorial" link. |

### Phase 2: Base Theme Styling (custom.css) (Estimated: 60 min)

**Goal**: Implement the specified dark and light mode color palettes and ensure smooth transitions.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P2-T01  | Medium | P1-T02       | `frontend/src/css/custom.css` | `custom.css` defines new `--ifm-color-primary` and background variables for dark mode. |
| P2-T02  | Medium | P2-T01       | `frontend/src/css/custom.css` | `custom.css` defines new `--ifm-color-primary` and background variables for light mode. |
| P2-T03  | Medium | P2-T02       | `frontend/src/css/custom.css` | CSS properties for smooth transitions (`transition: all 0.3s ease-in-out;`) are applied to relevant elements. |

### Phase 3: Custom Mode Toggle Implementation (Estimated: 90 min)

**Goal**: Develop and integrate the custom, animated dark/light mode toggle component.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P3-T01  | Medium | P2-T03       | `frontend/src/components/ThemeToggle/` (folder) | `ThemeToggle` component folder created. |
| P3-T02  | High   | P3-T01       | `frontend/src/components/ThemeToggle/index.tsx` | React component for toggle with pill shape, sun/moon icons, and basic functionality. |
| P3-T03  | High   | P3-T02       | `frontend/src/components/ThemeToggle/styles.module.css` | CSS for pill shape, glow effect, and sun/moon icon styling. |
| P3-T04  | High   | P3-T03       | `frontend/src/components/ThemeToggle/assets/` (folder + icons) | Sun and moon SVG/PNG icons are integrated; orbiting ring animation for moon is implemented. |
| P3-T05  | Medium | P3-T04       | `frontend/docusaurus.config.ts` | Custom `ThemeToggle` component replaces default Docusaurus theme toggle in navbar. |

### Phase 4: Polish Homepage and Docs Styling (Estimated: 60 min)

**Goal**: Apply the new theme consistently across the site and ensure a professional aesthetic.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P4-T01  | Medium | P3-T05       | `frontend/src/css/custom.css` | Homepage (index.tsx) and docs pages reflect the new color palette and theme. |
| P4-T02  | Medium | P4-T01       | (None)       | Visual inspection confirms "professional, balanced, 'wow' factor" aesthetic. |
| P4-T03  | Medium | P4-T02       | (None)       | No visual glitches or abrupt changes during theme switching. |

### Phase 5: Verification & Testing (Estimated: 45 min)

**Goal**: Ensure all UI changes are functional, visually consistent, and performant.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P5-T01  | Medium | P4-T03       | (None)       | Local build (`npm run build` in `frontend/`) completes without errors. |
| P5-T02  | High   | P5-T01       | (None)       | Manual visual verification of dark/light mode functionality, toggle animation, and overall aesthetic. |
| P5-T03  | High   | P5-T02       | (None)       | Lighthouse audit scores for Performance, Accessibility, Best Practices, and SEO are 90+ in both modes. |
| P5-T04  | Low    | P5-T03       | (None)       | Cross-browser compatibility check (simulated). |

## Risk Assessment & Mitigations

| Risk                                     | Mitigation                                                                                                                              |
|:-----------------------------------------|:----------------------------------------------------------------------------------------------------------------------------------------|
| Visual Inconsistencies                   | Use Docusaurus theming API; define variables centrally in `custom.css`; manual visual inspection across pages.                          |
| Complex Toggle Animation                 | Leverage existing CSS animation techniques; keep animation simple and performant; use SVG for icons for scalability.                   |
| Overriding Docusaurus Defaults           | Follow Docusaurus theming best practices (swizzling components if necessary, but prefer CSS overrides); test changes incrementally. |
| Lighthouse Score Degradation             | Optimize CSS/JavaScript; use performant animation techniques; lazy load non-critical assets; regular Lighthouse audits.                 |
| Time Consumption (Hackathon)             | Prioritize core theme elements; simplify toggle animation if time becomes a constraint; focus on the "wow" factor in key areas.      |

## Timeline Estimate (Hackathon-friendly)

-   **Phase 1 (Docusaurus Configuration & Navbar Updates)**: 30 minutes
-   **Phase 2 (Base Theme Styling)**: 60 minutes
-   **Phase 3 (Custom Mode Toggle Implementation)**: 90 minutes
-   **Phase 4 (Polish Homepage and Docs Styling)**: 60 minutes
-   **Phase 5 (Verification & Testing)**: 45 minutes
-   **Total Estimated Agent Time**: ~4.75 - 5 hours

## Verification & Success Criteria

-   **Local Build Success**: `npm run build` command in `frontend/` must complete without errors.
-   **Visual Consistency**: The new theme, including color palettes and typography, must be consistently applied across all pages in both dark and light modes.
-   **Toggle Functionality**: The custom toggle button must correctly switch between themes with the specified animation and icons.
-   **Lighthouse Scores**: Achieves Lighthouse scores of 90+ for Performance, Accessibility, Best Practices, and SEO.
-   **Aesthetic Approval**: The UI theme is perceived as modern, futuristic, professional, and visually appealing.

## Next Steps
The project is now ready for UI theme implementation.

**Next Recommended Command**: `/sp.implement` to execute the tasks outlined in this plan.
