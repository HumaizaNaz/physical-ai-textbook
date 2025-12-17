<!--
Sync Impact Report:
Version change: 1.0.0 → 1.0.0 (Re-establishment with more detail)
List of modified principles: All (updated with more detailed descriptions and new principles)
Added sections: Learning Outcomes (now fully populated), Technical Accuracy & Zero Hallucination (new principle), Co-Learning Pedagogy (new principle)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .gemini/commands/sp.adr.toml: ✅ updated
- .gemini/commands/sp.analyze.toml: ✅ updated
- .gemini/commands/sp.checklist.toml: ✅ updated
- .gemini/commands/sp.clarify.toml: ✅ updated
- .gemini/commands/sp.constitution.toml: ✅ updated
- .gemini/commands/sp.git.commit_pr.toml: ✅ updated
- .gemini/commands/sp.implement.toml: ✅ updated
- .gemini/commands/sp.phr.toml: ✅ updated
- .gemini/commands/sp.plan.toml: ✅ updated
- .gemini/commands/sp.specify.toml: ✅ updated
- .gemini/commands/sp.tasks.toml: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Textbook Constitution

## Preamble & Course Overview
This document establishes the foundational principles, standards, and guidelines for the Panaversity Hackathon project repository at https://github.com/HumaizaNaz/new-book. This repository is building an interactive AI-native textbook titled **Physical AI & Humanoid Robotics** using Docusaurus (frontend) and Spec-Kit Plus for spec-driven development.

**Course Title**: Physical AI & Humanoid Robotics
**Theme**: AI Systems in the Physical World – Embodied Intelligence
**Goal**: To bridge the gap between digital AI brains and physical humanoid bodies, educating students on the fusion of artificial intelligence with robotics. This Docusaurus-based resource aims to provide a comprehensive and practical learning experience.

**Core Tools**: ROS 2 (Iron/Humble), Gazebo, Unity, NVIDIA Isaac Sim/Lab 2025.1+, Nav2, OpenVLA, OpenAI Whisper, GPT models, FastAPI (optional backend), Docusaurus.

**Modules**:
1.  The Robotic Nervous System (ROS 2) – Weeks 3–5
2.  The Digital Twin (Gazebo & Unity) – Weeks 6–7
3.  The AI-Robot Brain (NVIDIA Isaac™) – Weeks 8–10
4.  Vision-Language-Action (VLA) + Capstone – Weeks 11–13

**Capstone**: Autonomous simulated humanoid accepting natural language voice commands.

## Core Mission
To educate and empower the next generation of engineers and researchers in the interdisciplinary field of Physical AI & Humanoid Robotics, fostering ethical development, robust safety, and intuitive human-robot interaction through practical, simulation-driven learning, bridging the gap between digital AI brains and physical humanoid bodies.

## Learning Outcomes
1.  Understand Physical AI principles and embodied intelligence.
2.  Master ROS 2 for robotic control.
3.  Simulate robots with Gazebo and Unity.
4.  Develop with NVIDIA Isaac AI robot platform.
5.  Design humanoid robots for natural interactions.
6.  Integrate GPT models for conversational robotics.

## Non-Negotiable Principles

### 1. Interdisciplinary Collaboration
Emphasize collaboration across AI, robotics, biomechanics, cognitive science, and ethics to foster a holistic understanding of Physical AI and Humanoid Robotics. This principle guides content creation to integrate diverse perspectives and ensure comprehensive coverage.

### 2. Ethical AI Development
Prioritize human well-being, autonomy, privacy, fairness, bias mitigation, transparency, and accountability in all aspects of AI and robotics development. All examples, discussions, and project guidelines must reflect a strong commitment to ethical considerations and human-centered design.

### 3. Robustness & Safety Engineering
Ensure the reliability and safety of Physical AI systems in unpredictable real-world environments through rigorous risk assessment, fault tolerance, and comprehensive testing methodologies. Practical exercises and case studies will highlight best practices in safety engineering.

### 4. Human-Robot Interaction Design
Focus on creating intuitive, natural, trustworthy, and effective human-robot interactions. Content will explore the psychological, social, and cultural factors influencing HRI, aiming for designs that enhance human capabilities and acceptance.

### 5. Continuous Learning & Adaptation
Promote the design and understanding of Physical AI systems capable of continuous learning and adaptation in dynamic, real-world deployment scenarios. This includes topics on reinforcement learning, adaptive control, and online learning strategies.

### 6. Technical Accuracy & Zero Hallucination
All content, code, and technical specifications must be 100% accurate. There shall be zero hallucination on technical details such as package names, ROS topics, URDF tags, API endpoints, tool versions, or any other technical specification.

### 7. Co-Learning Pedagogy
Adhere strictly to the defined Co-Learning teaching methodology, ensuring every lesson contains exactly three elements as specified in the Teaching Methodology section.

## Teaching Methodology
Every lesson MUST adhere to a structured Co-Learning approach, incorporating exactly three elements:
-   **1 Theory (💡 icon)**: Fundamental concepts and theoretical underpinnings.
-   **1 Key Insight (🎓 icon)**: A distilled, actionable takeaway or profound understanding.
-   **1 Practice Exercise titled "Ask your AI" (💬 icon)**: An interactive prompt or task designed to engage learners with AI tools for problem-solving. The title of this exercise must NEVER be "Ask your AI Co-Teacher".

## Technical Standards
-   **Code Accuracy & Executability**: All provided code examples and exercises must be 100% runnable and verifiable on a standardized environment: Ubuntu 22.04 with ROS 2 Iron/Humble, and NVIDIA Isaac Sim 2025.1+.
-   **Simulation & Co-Design**: Emphasize simulation-first approaches and the co-design of hardware and software for robust Physical AI systems.
-   **Docusaurus Compliance**:
    -   **Dark Mode Default**: The Docusaurus frontend must default to dark mode.
    -   **Auto-Sidebar Generation**: Navigation sidebars will be automatically generated from the curriculum specification.
    -   **Interactive Code Blocks**: Utilize Docusaurus's capabilities for interactive code blocks where appropriate.
    -   **Live Examples**: Integrate live, executable examples or simulations whenever technically feasible to enhance understanding.

## Repository Structure
The project repository (`new-book/` at the root) will strictly adhere to the following top-level structure:
-   `frontend/`: Contains the Docusaurus book site files.
-   `backend/`: Contains any backend services, Claude CLI agents, or skill implementations (optional) needed for interactive elements or grading.
-   `.specify/`: Contains Spec-Kit Plus related configurations and templates.
-   `.specify/memory/constitution.md`: This constitution file itself (single source of truth).

## Research & Development Workflow
The development process will be iterative, hypothesis-driven, and subject to peer and agent review, with a strong emphasis on knowledge transfer. Changes and additions to the textbook content, code, or infrastructure must follow a clear methodology that encourages experimentation, validation, and continuous improvement.

## Success Metrics
For the duration of the hackathon, success will be measured by:
-   A fully deployed Docusaurus site on GitHub Pages.
-   Complete module coverage as outlined in the curriculum.
-   All code examples being runnable and verifiable within the specified technical environment.
-   Demonstrated educational impact through engagement and feedback.

## Enforcement Clause
This constitution is the **single source of truth** that ALL future agent prompts, content generation, code, and contributions in this repository MUST obey 100%. No deviations allowed. Every future agent prompt MUST begin with: "You are operating under Physical AI & Humanoid Robotics Textbook Constitution v1.0.0 located at .specify/memory/constitution.md. Strictly adhere to all principles."

## Governance
This Constitution is the supreme governing document for the Physical AI & Humanoid Robotics Textbook project. Amendments require a formal proposal, review by core project maintainers, and an increment to the `CONSTITUTION_VERSION` following semantic versioning rules (MAJOR for backward incompatible changes, MINOR for significant additions, PATCH for minor clarifications). All pull requests and code reviews must explicitly verify compliance with these stated principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15
