# SP.Specify Theme Specification

## Context
This document specifies the custom Docusaurus theme, identified as `sp.specify-theme-v1`, which is designed to enhance the visual appearance and user experience of the "physical-ai-textbook" project. It details global typography, responsive adjustments, and a distinct color palette for both light and dark modes, incorporating "glass-style" elements. This theme replaces any previous custom CSS configurations to ensure a unified and branded look.

## Theme ID
`sp-specify-theme-v1`

## Global Markdown & Typography Enhancements
*   **Max Width:** Markdown content is constrained to `max-width: 750px` (with responsive adjustments), centered on the page, ensuring readability.
*   **Padding:** Horizontal padding applied to markdown content for spacing.
*   **Line Height:** Paragraphs (`p`) have a `line-height` of `1.7` for improved readability.
*   **Base Font Size:** `body` font size set to `16px`, with responsive adjustments down to `14px` on smaller screens.
*   **Headings:**
    *   `h1`: `2.4em` (`2.2em` on medium, `1.8em` on small screens) with adjusted margins.
    *   `h2`: `1.8em` with adjusted margins.
    *   `h3`: `1.4em` with adjusted margins.
*   **Lists/Paragraphs:** `font-size: 1em`.
*   **Code Blocks/Inline Code:** `font-size: 0.9em` with adjusted line height for `pre`.
*   **Images:** Responsive `max-width: 100%`, `height: auto`, centered.
*   **Font Family:** `Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif`.

## Color Palette - Light Mode (`:root`)

The light mode utilizes a clean, professional palette centered around blue and accent green.
*   **Primary Colors (`--ifm-color-primary` variants):**
    *   Base: `#0099cc` (a vibrant blue)
    *   Darker shades: `#0088b8`, `#0077a3`, `#00668f`
    *   Lighter shades: `#00aae0`, `#00bbf5`, `#33ccff`
*   **Accent Color (`--ifm-color-accent`):** `#00cc88` (a bright green, likely for success states or highlights)
*   **Background Color (`--ifm-background-color`):** `#f8fafc` (a very light off-white)
*   **Font Color Base (`--ifm-font-color-base`):** `#1e293b` (a dark charcoal/slate)
*   **Heading Color (`--ifm-heading-color`):** `#0f172a` (a very dark blue/black)
*   **Glass Elements:**
    *   `--glass-background`: `rgba(255, 255, 255, 0.85)` (semi-transparent white)
    *   `--glass-border`: `1px solid rgba(0, 153, 204, 0.2)` (light, semi-transparent primary blue border)
*   **Body Background:** A linear gradient `linear-gradient(135deg, #f8fafc 0%, #e2e8f0 50%, #cbd5e0 100%)` (subtle light gray to blue-gray gradient), with `background-attachment: fixed`.

## Color Palette - Dark Mode (`[data-theme='dark']`)

The dark mode switches to a deep, tech-inspired palette with prominent blues.
*   **Primary Colors (`--ifm-color-primary` variants):**
    *   Base: `#00d4ff` (a brighter, electric blue)
    *   Darker shades: `#00bde6`, `#00a6cc`
*   **Background Color (`--ifm-background-color`):** `#0a0e27` (a very dark, almost black, blue)
*   **Font Color Base (`--ifm-font-color-base`):** `#e8f4f8` (a very light, slightly bluish white)
*   **Glass Elements:**
    *   `--glass-background`: `rgba(21, 27, 61, 0.7)` (semi-transparent dark blue)
*   **Body Background:** A linear gradient `linear-gradient(135deg, #0a0e27 0%, #151b3d 50%, #1a1f42 100%)` (deep blue to darker blue gradient), with `!important` to override default.

## Glass Card Styling (`.glass-card`)
*   Provides a frosted glass effect with a semi-transparent background, `backdrop-filter: blur(16px)`, a light border, and rounded corners (`border-radius: 12px`).

## References
*   `frontend/src/css/custom.css` (where this theme will be applied)
*   `docusaurus-platform-spec.md`
*   `00-ADR.md`
