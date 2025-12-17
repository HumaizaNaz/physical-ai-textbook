---
id: 0001
title: Establish Official Constitution v1.0.0 for Physical AI & Humanoid Robotics Textbook
stage: constitution
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-or-grok
feature: textbook-generation-core
branch: master
user: HumaizaNaz
command: /sp.constitution
labels: ["constitution", "physical-ai", "humanoid-robotics", "hackathon", "panaversity", "docusaurus", "spec-kit-plus"]
links:
  spec: https://github.com/HumaizaNaz/new-book/blob/main/ (or null if not exists yet)
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests: null
---
## Prompt
You are creating the **official, immutable constitution v1.0.0** for the Panaversity Hackathon project repository at https://github.com/HumaizaNaz/new-book

This repository is building an interactive AI-native textbook titled **Physical AI & Humanoid Robotics** using Docusaurus (frontend) and Spec-Kit Plus for spec-driven development.

This constitution is the **single source of truth** that ALL future agent prompts, content generation, code, and contributions in this repository MUST obey 100%. No deviations allowed.

Key course details to incorporate:
- Course Title: Physical AI & Humanoid Robotics
- Theme: AI Systems in the Physical World – Embodied Intelligence
- Goal: Bridge the gap between digital AI brains and physical humanoid bodies
- Core Tools: ROS 2 (Iron/Humble), Gazebo, Unity, NVIDIA Isaac Sim/Lab 2025.1+, Nav2, OpenVLA, OpenAI Whisper, GPT models, FastAPI (optional backend), Docusaurus
- Modules: 
  1. The Robotic Nervous System (ROS 2) – Weeks 3–5
  2. The Digital Twin (Gazebo & Unity) – Weeks 6–7
  3. The AI-Robot Brain (NVIDIA Isaac™) – Weeks 8–10
  4. Vision-Language-Action (VLA) + Capstone – Weeks 11–13
- Capstone: Autonomous simulated humanoid accepting natural language voice commands
- Learning Outcomes (include verbatim):
  1. Understand Physical AI principles and embodied intelligence
  2. Master ROS 2 for robotic control
  3. Simulate robots with Gazebo and Unity
  4. Develop with NVIDIA Isaac AI robot platform
  5. Design humanoid robots for natural interactions
  6. Integrate GPT models for conversational robotics

Unbreakable Rules to Enforce:
1. Exactly **3 Co-Learning elements per lesson** (never more/less):
   - 1 Theory (💡 icon)
   - 1 Key Insight (🎓 icon)
   - 1 Practice Exercise titled **"Ask your AI"** (💬 icon)
2. 100% runnable, verifiable code on Ubuntu 22.04 + ROS 2 Iron/Humble + Isaac Sim 2025.1+
3. Zero hallucination on technical details (package names, ROS topics, URDF tags, API endpoints, tool versions)
4. Ethical, safe, and human-centered design must be emphasized in all relevant sections

Repository Structure (Sacred):
new-book/
└── spec-kit/
├── frontend/     ← Docusaurus book site
└── backend/      ← Claude CLI + Agents + Skills (optional)
└── .specify/
└── memory/
└── constitution.md   ← This file (single source of truth)
textOutput the complete constitution as a professional, beautifully formatted Markdown file with exactly these sections:

- Preamble & Course Overview (full title, theme, goal, high-level description)
- Core Mission
- Learning Outcomes (verbatim)
- Non-Negotiable Principles
  1. Interdisciplinary Collaboration
  2. Ethical AI Development
  3. Robustness & Safety Engineering
  4. Human-Robot Interaction Design
  5. Continuous Learning & Adaptation
  6. Technical Accuracy & Zero Hallucination
  7. Co-Learning Pedagogy (exactly 3 elements rule)
- Teaching Methodology
- Technical Standards (simulation, co-design, runnable code)
- Repository Structure
- Research & Development Workflow (iterative, hypothesis-driven, peer/agent review, knowledge transfer)
- Success Metrics (for hackathon: fully deployed Docusaurus site on GitHub Pages, complete module coverage, runnable examples, educational impact)
- Enforcement Clause (every future agent prompt MUST begin with: "You are operating under Physical AI & Humanoid Robotics Textbook Constitution v1.0.0 located at .specify/memory/constitution.md. Strictly adhere to all principles.")

Save the file exactly at: .specify/memory/constitution.md

After creation, confirm the constitution is established and suggest next steps:
- Create/update curriculum.yaml for auto-sidebar
- Scaffold Docusaurus in spec-kit/frontend/
- Begin generating Module 1 content

Constitution version: 1.0.0 – any change requires a new /sp.constitution command with incremented ID and version.
## Response
The constitution for the Physical AI & Humanoid Robotics Textbook project has been successfully re-established with comprehensive details and saved to `.specify/memory/constitution.md`.

**New Constitution Version**: 1.0.0 (Re-establishment with more detail)

**Files flagged for manual follow-up**:
- `.specify/templates/plan-template.md`: This template needs to be updated to align with the new, more detailed constitution, particularly its "Constitution Check" section.
- `.specify/templates/spec-template.md`: This template should be reviewed and updated to ensure that feature specifications align with the constitution's principles and technical standards.
- `.specify/templates/tasks-template.md`: This template needs to be reviewed and updated to reflect the constitution's principles in task breakdown and organization.

**Next Steps Recommended**:
1.  **Create/update `curriculum.yaml`**: Define the structure and content for the textbook, outlining modules, lessons, and co-learning elements as per the constitution, to enable auto-sidebar generation.
2.  **Scaffold Docusaurus**: Set up the basic Docusaurus site in the `frontend/` directory, adhering to the specified Docusaurus Compliance standards.
3.  **Begin Module 1 Content Generation**: Start developing the content for Module 1, ensuring strict adherence to the established constitution and its teaching methodology.

**Suggested Commit Message**:
```
docs: Re-established constitution for Physical AI & Humanoid Robotics Textbook v1.0.0 with full details
```