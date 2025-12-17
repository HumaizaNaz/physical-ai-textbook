# 012 - Updated Footer and Font Colors for SP.Specify Theme

## Context
Following the application of the new custom Docusaurus theme (`sp.specify-theme-v1`), it was necessary to explicitly define text colors for key UI elements (navbar, footer, headings) to ensure consistent theming and optimal contrast across both light and dark modes, based on the `sp.specify-theme-v1`'s defined color variables.

## Actions Taken
1.  **Modified `frontend/src/css/custom.css`**: Added specific CSS rules at the end of the file to target:
    *   `.navbar__title`, `.navbar__item`, `.navbar__link`
    *   `.footer__title`, `.footer__link-item`, `.footer__copyright`
    *   `a.menu__link`, `.theme-doc-sidebar-item-link`, `.theme-doc-sidebar-item-link.active`, `.theme-doc-sidebar-item-link:hover`
    *   `h1, h2, h3, h4, h5, h6`
    These elements now explicitly inherit their `color` from `var(--ifm-font-color-base)` or `var(--ifm-heading-color)` to align with the new theme's color palette.

## Outcome
The Docusaurus frontend now displays text in the navbar, footer, sidebar links, and headings with colors consistent with the `sp.specify-theme-v1`. This ensures proper readability and adherence to the custom theme's aesthetic in both light and dark modes, addressing the user's request for updated font colors.
