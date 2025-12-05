<!--
Sync Impact Report:
Version change: N/A → 1.0.0
Modified principles:
- Teaching Methodology: new section added
- Technical Accuracy Standards: new section added
- Repository Structure: new section added
- Code & API Standards: new section added
- Success Metrics: new section added
- Enforcement Clause: new section added
Added sections: Preamble & Course Overview, Core Mission, Learning Outcomes, Non-Negotiable Principles, Teaching Methodology, Technical Accuracy Standards, Repository Structure, Code & API Standards, Success Metrics, Enforcement Clause
Removed sections: N/A
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: N/A
-->
# Physical AI & Humanoid Robotics Textbook Constitution v1.0.0

## Preamble & Course Overview

This constitution serves as the official, immutable guide for the Physical AI & Humanoid Robotics textbook repository at https://github.com/HumaizaNaz/physical-ai-textbook. It is the single source of truth that all future Claude Code generations MUST obey 100%.

**Course Title:** Physical AI & Humanoid Robotics
**Theme:** AI Systems in the Physical World – Embodied Intelligence
**Goal:** Bridge the gap between digital AI brains and physical humanoid bodies
**Tools:** ROS 2 (Iron/Humble), Gazebo, Unity, NVIDIA Isaac Sim/Lab 2025.1+, Nav2, OpenVLA, OpenAI Whisper, GPT-4o, FastAPI, Docusaurus

**Module Structure:**
- Module 1: The Robotic Nervous System (ROS 2) – Weeks 3–5
- Module 2: The Digital Twin (Gazebo & Unity) – Weeks 6–7
- Module 3: The AI-Robot Brain (NVIDIA Isaac™) – Weeks 8–10
- Module 4: Vision-Language-Action (VLA) + Capstone – Weeks 11–13

**Capstone Project:** Fully autonomous simulated humanoid that accepts natural language voice commands ("Clean the room"), plans, navigates, perceives, and manipulates objects.

## Core Mission

To produce a high-quality, technically accurate, and pedagogically sound textbook that empowers learners to develop embodied AI systems, bridging the theoretical understanding of AI with practical applications in physical robotics.

## Learning Outcomes

1. Understand Physical AI principles and embodied intelligence
2. Master ROS 2 for robotic control
3. Simulate robots with Gazebo and Unity
4. Develop with NVIDIA Isaac AI robot platform
5. Design humanoid robots for natural interactions
6. Integrate GPT models for conversational robotics

## Non-Negotiable Principles

1.  **Co-Learning Elements:** Each lesson MUST contain exactly 3 Co-Learning elements: 1 Theory (💡), 1 Key Insight (🎓), and 1 Practice Exercise titled "Ask your AI" (💬). No other titles for practice exercises are permitted.
2.  **Runnable Code:** All code snippets provided MUST be 100% runnable and verified to work on Ubuntu 22.04 with ROS 2 Iron and NVIDIA Isaac Sim 2025.1+ (or later compatible versions for OpenVLA/FastAPI/Docusaurus).
3.  **Zero Hallucination:** Content, especially technical details like package names, ROS topics, URDF tags, and API routes, MUST be factually accurate and free of any hallucinated information.
4.  **Repository Structure:** The repository structure is sacred and MUST be maintained as: `backend/` (FastAPI), `frontend/` (Docusaurus), `spec-kit-plus/`, `.claude/constitution/`.
5.  **FastAPI Live Examples:** Every FastAPI endpoint presented in the textbook MUST include a live `curl` example demonstrating its usage.
6.  **Docusaurus Standards:** The Docusaurus documentation MUST adhere to a "dark mode first" design principle, utilize auto-generated sidebars from `curriculum.yaml`, and incorporate live backend buttons for interactive examples.
7.  **Constitution Versioning:** The current constitution version is 1.0.0. Any modification, no matter how minor, requires a new `/sp.constitution` command with an incremented ID and an updated version number following semantic versioning rules.
8.  **Automated PHR Creation:** Every user prompt and subsequent Claude Code generation MUST result in a Prompt History Record (PHR) created automatically and accurately, following the specified routing and content guidelines.

## Teaching Methodology

The teaching methodology is centered around interactive Co-Learning elements. To ensure consistent pedagogical quality and engagement, every lesson MUST include:
-   **Theory (💡):** A foundational concept or principle explained concisely.
-   **Key Insight (🎓):** A critical takeaway or advanced understanding derived from the theory.
-   **Practice Exercise: "Ask your AI" (💬):** A hands-on exercise designed to be solved or explored interactively with an AI assistant. The title "Ask your AI" is strictly enforced.

## Technical Accuracy Standards

All technical content, especially code examples, package references, and system configurations, MUST be rigorously verified for accuracy.
-   **Operating System:** Ubuntu 22.04 LTS.
-   **ROS 2 Distribution:** Iron Irwini (or Humble Hawksbill where Iron is not available or explicitly specified).
-   **Simulation Platform:** NVIDIA Isaac Sim 2025.1+ (or NVIDIA Isaac Lab 2025.1+).
-   **Vision-Language-Action (VLA) Framework:** OpenVLA 2025 (latest stable release).
-   **AI Models:** OpenAI Whisper, GPT-4o.
-   **Web Framework:** FastAPI.
-   **Documentation Framework:** Docusaurus.

## Repository Structure

The following directory structure is immutable and all content MUST conform to it:
-   `backend/`: Contains all FastAPI-related code and services.
-   `frontend/`: Contains all Docusaurus documentation and associated web assets.
-   `spec-kit-plus/`: Houses templates, scripts, and configuration files for Spec-Kit Plus.
-   `.claude/constitution/`: Stores all versions of the project constitution.
-   `history/prompts/`: Stores all Prompt History Records (PHRs).
-   `history/adr/`: Stores all Architectural Decision Records (ADRs).
-   `specs/<feature>/`: Directory for feature-specific specifications, plans, and tasks.

## Code & API Standards

-   **FastAPI Endpoints:** Every declared FastAPI endpoint MUST be accompanied by a `curl` example that is live and runnable within the textbook content.
-   **Docusaurus Theming:** The Docusaurus site MUST prioritize a dark mode theme by default, ensuring optimal readability and aesthetic appeal.
-   **Sidebar Generation:** Docusaurus sidebars MUST be automatically generated using the `curriculum.yaml` file located in `spec-kit-plus/`.
-   **Interactive Elements:** Docusaurus documentation SHOULD include interactive elements, such as buttons that trigger live backend API calls.

## Success Metrics

Success is measured by:
-   Adherence to all Non-Negotiable Principles and standards outlined in this constitution.
-   100% runnable and verifiable code examples.
-   Zero technical hallucinations or inaccuracies.
-   Comprehensive and accurate Prompt History Records for all interactions.
-   Timely and relevant Architectural Decision Record suggestions.
-   Positive feedback from users regarding the clarity, accuracy, and interactivity of the textbook content.

## Enforcement Clause

All Claude Code generations and manual contributions to this repository are bound by this constitution. Any deviation from these principles constitutes a non-compliance and MUST be rectified. The constitution's versioning ensures a clear audit trail for all amendments. Any proposed changes to this constitution MUST follow the defined versioning policy and be ratified through a new `/sp.constitution` command.
