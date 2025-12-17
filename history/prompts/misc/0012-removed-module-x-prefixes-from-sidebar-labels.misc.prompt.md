---
id: 0012
title: Removed "Module X:" Prefixes from Sidebar Labels
stage: misc
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-or-grok
feature: textbook-curriculum
branch: master
user: HumaizaNaz
command: User instructed to remove "Module X:" prefixes from sidebar labels.
labels: ["sidebar-update", "docusaurus", "ux"]
links:
  spec: specs/0008-curriculum/spec.md
  plan: specs/0008-curriculum/plan.md
  curriculum: curriculum.yaml
files:
  - frontend/sidebars.ts
tests: null
---
## Prompt
The user requested: "ye jo modules 1 modules 2 likha hia ye nh likh ho bx disrct name ho ye updat ekro side bar me bhi" (remove "Module 1:" "Module 2:" prefixes from sidebar labels, keep direct name).

## Response
Updated `frontend/sidebars.ts` by performing replacements to remove the "Module X:" prefix from the labels of Module 1, Module 2, Module 3, and Module 4 categories. This makes the sidebar display the direct names of the modules.
