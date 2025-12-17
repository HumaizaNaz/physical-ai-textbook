---
id: 0017
title: Updated GitHub Links in Docusaurus Configuration
stage: general
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-or-grok
feature: textbook-ui-theme
branch: master
user: HumaizaNaz
command: User requested to update GitHub and other links in `docusaurus.config.ts`.
labels: ["config", "links", "docusaurus"]
links:
  spec: specs/0012-ui-theme/spec.md
  plan: specs/0012-ui-theme/plan.md
files:
  - frontend/docusaurus.config.ts
tests: null
---
## Prompt
The user requested: "ok now docusaurus.config.ts me links update kro github wagera sb ki" (update GitHub and other links in docusaurus.config.ts).

## Response
Updated all GitHub-related links in `frontend/docusaurus.config.ts`:
-   `editUrl` in `presets.docs` to `https://github.com/HumaizaNaz/new-book/tree/main/frontend/`
-   `href` in `navbar.items` to `https://github.com/HumaizaNaz/new-book`
-   `href` in `footer.links` to `https://github.com/HumaizaNaz/new-book`
These changes ensure all links point to the user's repository.
