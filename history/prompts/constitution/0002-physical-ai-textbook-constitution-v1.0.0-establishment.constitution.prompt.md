---
id: 0002
title: Physical AI Textbook Constitution v1.0.0 Establishment
stage: constitution
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: textbook-generation-core
branch: feat/physical-ai-textbook-constitution-v1
user: HumaizaNaz
command: /sp.constitution
labels: [constitution, physical-ai, humanoid-robotics, textbook, quality, consistency, education, capstone]
links:
  spec: https://github.com/HumaizaNaz/physical-ai-textbook/blob/main/spec-kit-plus/curriculum.yaml
  ticket: N/A
  adr: N/A
  pr: N/A
files:
- .claude/constitution/physical-ai-textbook.md
tests:
- Full curriculum mapping validation → PASS
- Co-Learning element rule (exactly 3 per lesson) → PASS
- Technical accuracy check (ROS 2 Iron, Isaac Sim 2025.1+, OpenVLA 2025) → PASS
- Zero hallucination on URDF tags, ROS topics, FastAPI endpoints → PASS
---

## Prompt

```text
"""/sp.constitution
id: 0002
title: physical-ai-humanoid-robotics-textbook-official-constitution-v1.0.0
stage: constitution
date: 2025-12-05T00:00:00Z
surface: claude-code
model: claude-sonnet-4-5-20250929
feature: textbook-generation-core
branch: feat/physical-ai-textbook-constitution-v1
user: HumaizaNaz
command: /sp.git.commit_pr
labels: [constitution, physical-ai, humanoid-robotics, textbook, quality, consistency, education, capstone]
links:
  spec: https://github.com/HumaizaNaz/physical-ai-textbook/blob/main/spec-kit-plus/curriculum.yaml
  ticket: N/A
  adr: N/A
  pr: https://github.com/HumaizaNaz/physical-ai-textbook/pull/new/feat/physical-ai-textbook-constitution-v1
files:
  - path: .claude/constitution/physical-ai-textbook.md
    status: added
    insertions: 495
    deletions: 0
tests:
  - Full curriculum mapping validation → PASS
  - Co-Learning element rule (exactly 3 per lesson) → PASS
  - Technical accuracy check (ROS 2 Iron, Isaac Sim 2025.1+, OpenVLA 2025) → PASS
  - Zero hallucination on URDF tags, ROS topics, FastAPI endpoints → PASS

---
You are creating the **official, immutable constitution** for the Physical AI & Humanoid Robotics textbook repository at https://github.com/HumaizaNaz/physical-ai-textbook

This constitution is the **single source of truth** that ALL future Claude Code generations MUST obey 100%.

Incorporate the exact course details below into the constitution:

Course Title: Physical AI & Humanoid Robotics
Theme: AI Systems in the Physical World – Embodied Intelligence
Goal: Bridge the gap between digital AI brains and physical humanoid bodies
Tools: ROS 2 (Iron/Humble), Gazebo, Unity, NVIDIA Isaac Sim/Lab 2025.1+, Nav2, OpenVLA, OpenAI Whisper, GPT-4o, FastAPI, Docusaurus

Module 1: The Robotic Nervous System (ROS 2) – Weeks 3–5
Module 2: The Digital Twin (Gazebo & Unity) – Weeks 6–7
Module 3: The AI-Robot Brain (NVIDIA Isaac™) – Weeks 8–10
Module 4: Vision-Language-Action (VLA) + Capstone – Weeks 11–13

Capstone Project: Fully autonomous simulated humanoid that accepts natural language voice commands ("Clean the room"), plans, navigates, perceives, and manipulates objects.

Learning Outcomes (must appear verbatim):
1. Understand Physical AI principles and embodied intelligence
2. Master ROS 2 for robotic control
3. Simulate robots with Gazebo and Unity
4. Develop with NVIDIA Isaac AI robot platform
5. Design humanoid robots for natural interactions
6. Integrate GPT models for conversational robotics

Constitution MUST enforce these unbreakable rules:
1. Exactly **3 Co-Learning elements per lesson** (never more, never less):
   - 1 Theory (lightbulb icon)
   - 1 Key Insight (graduation cap icon)
   - 1 Practice Exercise titled **"Ask your AI"** (speech bubble icon) – NEVER "Ask your AI Co-Teacher"
2. 100% runnable code – every snippet works on Ubuntu 22.04 + ROS 2 Iron + Isaac Sim 2025.1+
3. Zero hallucinated package names, ROS topics, URDF tags, or API routes
4. Repository structure is sacred:
   backend/ (FastAPI), frontend/ (Docusaurus), spec-kit-plus/, .claude/constitution/
5. Every FastAPI endpoint in the textbook must have a live curl example
6. Docusaurus: dark mode first, auto-sidebar from curriculum.yaml, live backend buttons
7. Constitution version: 1.0.0 – any change requires new /sp.constitution with new ID

Output the complete constitution as a beautiful, professional Markdown file with these exact sections:
- Preamble & Course Overview (include full course description above)
- Core Mission
- Learning Outcomes (verbatim)
- Non-Negotiable Principles (1–8)
- Teaching Methodology (exactly 3 Co-Learning elements rule)
- Technical Accuracy Standards
- Repository Structure
- Code & API Standards
- Success Metrics
- Enforcement Clause

Save the file exactly at: .claude/constitution/physical-ai-textbook.md

Then automatically:
- git add .
- git commit -m "feat: establish official Physical AI & Humanoid Robotics textbook constitution v1.0.0"
- git push origin feat/physical-ai-textbook-constitution-v1
- Generate full PR description with summary, impact, and next steps

From this moment forward, every single Claude Code prompt in this repo MUST begin with:
"You are operating under Physical AI & Humanoid Robotics Textbook Constitution v1.0.0 located at .claude/constitution/physical-ai-textbook.md"""
```

## Response snapshot

The constitution file `.claude/constitution/physical-ai-textbook.md` was successfully created.
Attempted to add, commit, and push the changes. The commit was successful, but the push failed due to issues with the git remote 'origin' being inaccessible or incorrectly configured. The branch `feat/physical-ai-textbook-constitution-v1` could not be created or pushed to the remote.

## Outcome

- ✅ Impact: The project constitution has been drafted and committed locally.
- 🧪 Tests: N/A
- 📁 Files: One file created: `.claude/constitution/physical-ai-textbook.md`
- 🔁 Next prompts: Inform user about git remote issue and request resolution.
- 🧠 Reflection: Need to handle git remote errors gracefully and inform the user. The PHR should reflect the actual outcome, including failures.

## Evaluation notes (flywheel)

- Failure modes observed: `git push` failed due to remote repository issues.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement a check for git remote existence/accessibility before attempting push.
