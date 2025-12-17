# Feature Specification: Curriculum Structure and Content Outline for Physical AI & Humanoid Robotics Textbook

**Feature ID**: 0008
**Branch**: 0008-curriculum
**Status**: Ready for Implementation

## Preamble & Course Overview

**Course Title**: Physical AI & Humanoid Robotics
**Focus and Theme**: AI Systems in the Physical World. Embodied Intelligence.
**Goal**: Bridging the gap between the digital brain and the physical body. Students apply their AI knowledge to control Humanoid Robots in simulated and real-world environments.

**Quarter Overview**: The future of AI extends beyond digital spaces into the physical world. This capstone quarter introduces Physical AI—AI systems that function in reality and comprehend physical laws. Students learn to design, simulate, and deploy humanoid robots capable of natural human interactions using ROS 2, Gazebo, and NVIDIA Isaac.

**Core Tools**: ROS 2 (Iron/Humble), Gazebo, Unity, NVIDIA Isaac Sim/Lab 2025.1+, Nav2, OpenVLA, OpenAI Whisper, GPT models.

**Why Physical AI Matters**: Humanoid robots are poised to excel in our human-centered world because they share our physical form and can be trained with abundant data from interacting in human environments. This represents a significant transition from AI models confined to digital environments to embodied intelligence that operates in physical space.

## Learning Outcomes (verbatim)

1.  Understand Physical AI principles and embodied intelligence
2.  Master ROS 2 (Robot Operating System) for robotic control
3.  Simulate robots with Gazebo and Unity
4.  Develop with NVIDIA Isaac AI robot platform
5.  Design humanoid robots for natural interactions
6.  Integrate GPT models for conversational robotics

## Content Hierarchy (Full List of Proposed Docs Files and Folders)

This section outlines the planned Docusaurus documentation structure. All paths are relative to `frontend/docs/`.

**Introduction (Weeks 1-2)**
*   `introduction/01-foundations-of-physical-ai.md`
*   `introduction/02-embodied-intelligence.md`
*   `introduction/03-sensor-systems.md`
*   `introduction/04-hardware-requirements.md` (FR-005)
*   `introduction/05-sim-to-real-tips.md` (FR-006)
*   `introduction/06-ethical-hri-safety.md` (FR-007)

**Module 1: The Robotic Nervous System (ROS 2) (Weeks 3-5)**
*   `module-1-ros2/01-ros2-architecture.md`
*   `module-1-ros2/02-nodes-topics.md`
*   `module-1-ros2/03-services-actions.md`
*   `module-1-ros2/04-building-ros2-packages-python.md`
*   `module-1-ros2/05-launch-files-parameters.md`
*   `module-1-ros2/06-urdf-for-humanoids.md`

**Module 2: The Digital Twin (Gazebo & Unity) (Weeks 6-7)**
*   `module-2-digital-twin/01-gazebo-setup.md`
*   `module-2-digital-twin/02-urdf-sdf-formats.md`
*   `module-2-digital-twin/03-physics-sensor-simulation.md`
*   `module-2-digital-twin/04-unity-visualization.md`

**Module 3: The AI-Robot Brain (NVIDIA Isaac™) (Weeks 8-10)**
*   `module-3-isaac/01-isaac-sdk-isaac-sim.md`
*   `module-3-isaac/02-ai-perception-manipulation.md`
*   `module-3-isaac/03-reinforcement-learning-robot-control.md`
*   `module-3-isaac/04-sim-to-real-transfer.md`

**Module 4: Vision-Language-Action (VLA) (Weeks 11-13)**
*   `module-4-vla/01-humanoid-kinematics-dynamics.md` (from "Humanoid Robot Development" in Weekly Breakdown)
*   `module-4-vla/02-bipedal-locomotion-balance.md`
*   `module-4-vla/03-manipulation-grasping.md`
*   `module-4-vla/04-natural-human-robot-interaction.md`
*   `module-4-vla/05-gpt-conversational-ai.md` (from "Conversational Robotics" in Weekly Breakdown)
*   `module-4-vla/06-speech-recognition-nlu.md`
*   `module-4-vla/07-multi-modal-interaction.md`
*   `module-4-vla/08-voice-to-action-whisper.md` (from Module 4 description)
*   `module-4-vla/09-cognitive-planning-llms.md` (from Module 4 description)

**Capstone Project: The Autonomous Humanoid**
*   `capstone/01-capstone-project-overview.md`
*   `capstone/02-capstone-rubric.md` (FR-012)

## Co-Learning Structure Enforcement

Every lesson page MUST strictly adhere to the "exactly 3 co-learning elements per lesson" rule as defined in the Constitution v1.0.0. Each element must be clearly demarcated with its respective icon (💡 Theory, 🎓 Key Insight, 💬 Ask your AI) and follow the specified naming for the practice exercise ("Ask your AI").

## Technical Standards

-   **Runnable Code**: All code snippets included in lessons MUST be 100% runnable and verifiable on the specified technical stack: Ubuntu 22.04 + ROS 2 Iron/Humble + Isaac Sim 2025.1+.
-   **Zero Hallucination**: All technical details, including but not limited to URDF tags, ROS topics, API endpoints, tool versions, and code logic, MUST be factually accurate and free from any form of hallucination.
-   **Docusaurus Compliance**: The site MUST default to dark mode and maintain a professional aesthetic as per the Constitution.

## User Stories & Acceptance Criteria

### User Story 1 - Structured Learning Path (Priority: P1)

As a student, I want to easily navigate through the textbook's modules and lessons, so that I can follow the learning path efficiently and understand the progression of topics.

**Acceptance Scenarios**:

1.  **Given** I am on the textbook's homepage, **When** I view the sidebar, **Then** I see a clear hierarchy of an Introduction, 4 main Modules, and a Capstone section.
2.  **Given** I select a Module from the sidebar, **When** the Module's page loads, **Then** all lessons within that Module are listed in a sequential and logical order.
3.  **Given** I navigate through the lessons, **When** I move from one lesson to the next, **Then** the progression of topics feels natural and coherent.

### User Story 2 - Engaging Co-Learning Experience (Priority: P1)

As a student, I want every lesson to include exactly three co-learning elements (💡 Theory, 🎓 Key Insight, 💬 Ask your AI), so that I can engage with the material effectively, deepen my understanding, and apply my knowledge through interactive prompts.

**Acceptance Scenarios**:

1.  **Given** I am viewing any lesson page, **When** I scroll through the content, **Then** I find exactly one section marked 💡 Theory, one section marked 🎓 Key Insight, and one section marked 💬 Ask your AI.
2.  **Given** I interact with the 💬 Ask your AI exercise, **When** I read its instructions, **Then** it provides a clear prompt for using an AI assistant for further exploration.
3.  **Given** I am reviewing a lesson, **When** I check the presence of co-learning elements, **Then** there are never more or fewer than three co-learning elements of the specified types.

### User Story 3 - Practical and Verifiable Learning (Priority: P2)

As a student, I want to find verifiable and runnable code snippets within the lessons for ROS 2 Python, URDF, Gazebo plugins, and Isaac Sim launches, so that I can practice and confirm my understanding in a practical and hands-on way.

**Acceptance Scenarios**:

1.  **Given** I am viewing a lesson that discusses code, **When** I find a code snippet, **Then** the snippet is presented in a clear, copyable format (e.g., Docusaurus code block).
2.  **Given** I copy a ROS 2 Python code snippet, **When** I run it in a compatible environment, **Then** it executes without syntax errors and produces the expected output.
3.  **Given** I copy a Gazebo plugin or Isaac Sim launch configuration, **When** I integrate it into the respective simulation environment, **Then** it functions as described in the lesson.

## Success Criteria (mandatory - measurable)

-   **SC-001**: Every generated lesson page MUST contain exactly one 💡 Theory element, one 🎓 Key Insight element, and one 💬 Ask your AI (practice exercise) element.
-   **SC-002**: The Docusaurus sidebar MUST correctly reflect the structured curriculum (Introduction, 4 Modules, Capstone), with all lessons organized logically, generated via `curriculum.yaml` (when implemented) or `sidebars.js`.
-   **SC-003**: All code snippets within the textbook, when copied and executed in the specified environment (Ubuntu 22.04 + ROS 2 Iron/Humble + Isaac Sim 2025.1+), MUST run without errors and produce expected results.
-   **SC-004**: The deployed Docusaurus site, containing the full curriculum, MUST adhere to dark mode default and technical accuracy principles as per the Constitution, achieving Lighthouse scores of 90+ across all categories.
-   **SC-005**: The curriculum MUST cover all specified sections: Introduction (including Hardware, Sim-to-Real, Ethical HRI), 4 main Modules, and Capstone (including rubric).
-   **SC-006**: The textbook content, including code snippets and conceptual explanations, MUST demonstrate zero hallucination and high technical accuracy.