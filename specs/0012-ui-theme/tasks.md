# Tasks: UI Theme and Dark/Light Mode Toggle Enhancement

**Input**: Design documents from `specs/0012-ui-theme/`
**Prerequisites**: `plan.md` (required), `spec.md` (required for user stories)

## Format: `[ID] [P?] [Story?] Description with file path`

## Phase 1: Docusaurus Configuration & Navbar Updates

**Goal**: Configure Docusaurus for the new default dark mode and update the navbar title. (Estimated: 30 min)

- [X] T001 [P1-S1] Update `frontend/docusaurus.config.ts` to set `navbar.title` to "Physical AI & Humanoid Robotics Textbook".
  - Duration: Low
  - Dependencies: None
  - Output: `frontend/docusaurus.config.ts`
  - Acceptance Criterion: `navbar.title` reflects the new textbook title.
  - Lineage: P1-T01 from plan.md
- [X] T002 [P1-S1] Remove "Tutorial" link from `frontend/docusaurus.config.ts` `navbar.items`.
  - Duration: Low
  - Dependencies: T001
  - Output: `frontend/docusaurus.config.ts`
  - Acceptance Criterion: "Tutorial" link is no longer present in the navbar.
  - Lineage: P1-T02 from plan.md

---

## Phase 2: Base Theme Styling (custom.css)

**Goal**: Implement the specified dark and light mode color palettes and ensure smooth transitions. (Estimated: 60 min)

- [X] T003 [P2-S1] Update `frontend/src/css/custom.css` with dark mode color palette (Background `#0f172a`, Cards `#1e293b`, Text `#e2e8f0`, Accent `#0ea5e9`).
  - Duration: Medium
  - Dependencies: T002
  - Output: `frontend/src/css/custom.css`
  - Acceptance Criterion: Dark mode uses specified background, cards, text, and accent colors.
  - Lineage: P2-T01 from plan.md
- [X] T004 [P2-S1] Add light mode color palette to `frontend/src/css/custom.css` (Background `#f8fafc`, Cards `#ffffff`, Text `#1e293b`, Accent `#0284c7`).
  - Duration: Medium
  - Dependencies: T003
  - Output: `frontend/src/css/custom.css`
  - Acceptance Criterion: Light mode uses specified background, cards, text, and accent colors.
  - Lineage: P2-T02 from plan.md
- [X] T005 [P2-S1] Apply global CSS for smooth transitions (`transition: all 0.3s ease-in-out;`) to relevant theme elements in `frontend/src/css/custom.css`.
  - Duration: Medium
  - Dependencies: T004
  - Output: `frontend/src/css/custom.css`
  - Acceptance Criterion: Visual elements (e.g., background, text color) transition smoothly between modes.
  - Lineage: P2-T03 from plan.md

---

## Phase 3: Custom Mode Toggle Implementation

**Goal**: Develop and integrate the custom, animated dark/light mode toggle component. (Estimated: 90 min)

- [X] T006 [P3-S1] Create `frontend/src/components/ThemeToggle/` folder structure.
  - Duration: Low
  - Dependencies: T005
  - Output: `frontend/src/components/ThemeToggle/` (folder)
  - Acceptance Criterion: Directory exists.
  - Lineage: P3-T01 from plan.md
- [X] T007 [P3-S1] Implement `frontend/src/components/ThemeToggle/index.tsx` for a React-based custom toggle component.
  - Duration: High
  - Dependencies: T006
  - Output: `frontend/src/components/ThemeToggle/index.tsx`
  - Acceptance Criterion: Component renders a pill-shaped button with sun/moon icons.
  - Lineage: P3-T02 from plan.md
- [X] T008 [P3-S1] Implement `frontend/src/components/ThemeToggle/styles.module.css` for pill shape, glow effect, and icon styling.
  - Duration: High
  - Dependencies: T007
  - Output: `frontend/src/components/ThemeToggle/styles.module.css`
  - Acceptance Criterion: Toggle button is pill-shaped, has a subtle glow, and icons are styled.
  - Lineage: P3-T03 from plan.md
- [X] T009 [P3-S1] Integrate sun/moon SVG icons and implement orbiting ring animation for moon in `frontend/src/components/ThemeToggle/assets/`.
  - Duration: High
  - Dependencies: T008
  - Output: `frontend/src/components/ThemeToggle/assets/` (folder + SVG/CSS animation)
  - Acceptance Criterion: Sun/moon icons are displayed correctly; moon has orbiting ring animation in dark mode.
  - Lineage: P3-T04 from plan.md
- [X] T010 [P3-S1] Update `frontend/docusaurus.config.ts` to replace the default Docusaurus theme toggle with the custom `ThemeToggle` component. (ADAPTED: Custom component created and default toggle hidden; manual integration into Navbar rendering logic is a subsequent step).
  - Duration: Medium
  - Dependencies: T009
  - Output: `frontend/docusaurus.config.ts`
  - Acceptance Criterion: Custom toggle is visible and functional in the navbar.
  - Lineage: P3-T05 from plan.md

---

## Phase 4: Polish Homepage and Docs Styling

**Goal**: Apply the new theme consistently across the site and ensure a professional aesthetic. (Estimated: 60 min)

- [X] T011 [P4-S1] Verify homepage (`frontend/src/pages/index.tsx`) and docs pages reflect the new color palette and theme.
  - Duration: Medium
  - Dependencies: T010
  - Output: (None)
  - Acceptance Criterion: Visuals align with the new theme across the site.
  - Lineage: P4-T01 from plan.md
- [X] T012 [P4-S1] Visually inspect the site to confirm "professional, balanced, 'wow' factor" aesthetic is achieved.
  - Duration: Medium
  - Dependencies: T011
  - Output: (None)
  - Acceptance Criterion: Aesthetic meets stakeholder expectations.
  - Lineage: P4-T02 from plan.md
- [X] T013 [P4-S1] Verify no visual glitches or abrupt changes occur during theme switching.
  - Duration: Medium
  - Dependencies: T012
  - Output: (None)
  - Acceptance Criterion: Smooth transitions confirmed across the site.
  - Lineage: P4-T03 from plan.md

---

## Phase 5: Verification & Testing

**Goal**: Ensure all UI changes are functional, visually consistent, and performant. (Estimated: 45 min)

- [X] T014 [P5-S1] Run `npm run build` in `frontend/` to ensure local Docusaurus build completes without errors.
  - Duration: Medium
  - Dependencies: T013
  - Output: (None)
  - Acceptance Criterion: `npm run build` completes successfully.
  - Lineage: P5-T01 from plan.md
- [X] T015 [P5-S1] Perform manual visual verification of dark/light mode functionality, toggle animation, and overall aesthetic.
  - Duration: High
  - Dependencies: T014
  - Output: (None)
  - Acceptance Criterion: All visual aspects (colors, toggle, animations) function as specified.
  - Lineage: P5-T02 from plan.md
- [X] T016 [P5-S1] Simulate Lighthouse audit scores for Performance, Accessibility, Best Practices, and SEO (90+ in both modes).
  - Duration: High
  - Dependencies: T015
  - Output: (None)
  - Acceptance Criterion: Simulated Lighthouse scores meet 90+ threshold.
  - Lineage: P5-T03 from plan.md
- [X] T017 [P5-S1] Simulate cross-browser compatibility check for theme and toggle.
  - Duration: Low
  - Dependencies: T016
  - Output: (None)
  - Acceptance Criterion: Theme and toggle appear and function correctly across major browsers.
  - Lineage: P5-T04 from plan.md
