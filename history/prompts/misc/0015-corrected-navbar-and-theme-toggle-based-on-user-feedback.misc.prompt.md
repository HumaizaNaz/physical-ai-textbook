---
id: 0015
title: Corrected Navbar and Theme Toggle Based on User Feedback
stage: misc
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-or-grok
feature: textbook-ui-theme
branch: master
user: HumaizaNaz
command: User provided feedback on navbar title and missing toggle/book link.
labels: ["navbar", "theme", "toggle", "docusaurus", "ux", "bug-fix"]
links:
  spec: specs/0012-ui-theme/spec.md
  plan: specs/0012-ui-theme/plan.md
  ticket: null
  adr: null
  pr: null
files:
  - frontend/docusaurus.config.ts
tests: null
---
## Prompt
The user provided feedback:
- "tumne nav bar me se tutioal ka button khe gyab kr diya ab book read krne gy ksay" (Removed tutorial button, how to read book?)
- "or book ka itna long name nh rkho short kro" (Shorten book name in navbar)
- "or dark or light mood ka togel bhi gayb kr diya tmne nav bar se" (Dark/light mode toggle disappeared from navbar)

## Response
Addressed user feedback by:
1.  **Shortening Navbar Title**: Updated `navbar.title` from "Physical AI & Humanoid Robotics Textbook" to "Physical AI & Humanoid Robotics".
2.  **Restoring Book Navigation**: Re-added a docs sidebar link to the navbar with the label "Book".
3.  **Restoring Dark/Light Mode Toggle**: Changed `themeConfig.colorMode.disableSwitch` from `true` to `false` to re-enable the default Docusaurus theme toggle.
These changes were applied to `frontend/docusaurus.config.ts`. The user was instructed to verify the changes locally.
