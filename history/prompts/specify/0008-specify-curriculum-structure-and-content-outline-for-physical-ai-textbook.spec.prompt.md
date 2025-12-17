---
id: 0008
title: Specify Curriculum Structure and Content Outline for Physical AI Textbook
stage: spec
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-or-grok
feature: textbook-curriculum
branch: master
user: HumaizaNaz
command: /sp.specify
labels: ["spec", "curriculum", "docusaurus", "modules", "co-learning", "hackathon", "panaversity"]
links:
  spec: specs/0008-curriculum/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/0008-curriculum/spec.md
  - specs/0008-curriculum/checklists/requirements.md
tests: null
---
## Prompt
The user wants to specify the curriculum structure and content outline for the Physical AI Textbook. The textbook is built in the existing Docusaurus site at `frontend/`.

**Course Details** (used from Constitution):
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
- Learning Outcomes:
  1. Understand Physical AI principles and embodied intelligence
  2. Master ROS 2 for robotic control
  3. Simulate robots with Gazebo and Unity
  4. Develop with NVIDIA Isaac AI robot platform
  5. Design humanoid robots for natural interactions
  6. Integrate GPT models for conversational robotics

**Requirements**:
- Structure the textbook into 4 main modules + Introduction + Capstone.
- Break weekly topics into individual Docusaurus docs pages (e.g., docs/module1/week3-ros-architecture.md).
- Every lesson page MUST have exactly 3 co-learning elements: 💡 Theory, 🎓 Key Insight, 💬 Ask your AI (practice exercise).
- Include verifiable, runnable code snippets (ROS 2 Python, URDF, Gazebo plugins, Isaac Sim launches) – zero hallucination.
- Add missing features from previous history: Hardware requirements table, sim-to-real tips, ethical considerations in HRI.
- Optional enhancements: Interactive quizzes (placeholder), diagram placeholders, capstone rubric.
- Auto-generate sidebar via `curriculum.yaml` or `sidebars.js`.
- All prompts/logs in English, saved in history.
## Response
The detailed curriculum specification for the Physical AI & Humanoid Robotics Textbook has been successfully created and saved to `specs/0008-curriculum/spec.md`.

**Specification File**: `specs/0008-curriculum/spec.md`
**Quality Checklist**: `specs/0008-curriculum/checklists/requirements.md` (all checks passed)

The specification comprehensively outlines the textbook's structure into an Introduction, 4 Modules, and a Capstone section, detailing user stories, functional and non-functional requirements, key entities, and measurable success criteria. It also includes the content outline based on the course details and ensures adherence to the Constitution's principles, such as the "exactly 3 co-learning elements per lesson" rule and technical accuracy.

**Next Recommended Command**: `/sp.plan` to create the implementation plan for generating the actual MDX pages and integrating the new content.
