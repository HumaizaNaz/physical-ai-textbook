# Implementation Plan: Curriculum Integration and Content Generation

**Branch**: `0008-curriculum` | **Date**: 2025-12-16 | **Spec**: [specs/0008-curriculum/spec.md](specs/0008-curriculum/spec.md)
**Input**: Feature specification from `specs/0008-curriculum/spec.md` and curriculum structure from `curriculum.yaml`

## Overview & Goals
This plan outlines the steps to integrate the defined curriculum structure into the existing Docusaurus site and generate all missing MDX/Markdown lesson files. The primary goals are to ensure the textbook's content adheres to the Constitution's principles (especially co-learning elements and runnable code), provides a clear navigation path, and is ready for deployment. This phase focuses on content creation and structural setup within the Docusaurus framework.

## Summary
The "Curriculum Integration and Content Generation" feature will populate the Docusaurus site with the full textbook content, structured into modules and lessons as defined in the `curriculum.yaml` and `specs/0008-curriculum/spec.md`. It emphasizes the generation of content with exactly three co-learning elements per lesson and includes runnable code snippets, hardware requirements, ethical considerations, and assessments.

## Technical Context

**Language/Version**: Node.js v18 LTS (npm), Python 3.x, JavaScript/TypeScript (Docusaurus)
**Primary Dependencies**: Docusaurus v3, npm, ROS 2, Gazebo, Unity, NVIDIA Isaac Sim/ROS, LLMs (for content generation/co-learning exercises).
**Storage**: Docusaurus Markdown/MDX files (`.md`, `.mdx`) within `frontend/docs/`.
**Testing**: Local Docusaurus build, content validation (co-learning elements, code snippets), sidebar navigation, Lighthouse audits.
**Target Platform**: Web (GitHub Pages, Docusaurus static site).
**Project Type**: Web application (Static Site Generator - Textbook).
**Performance Goals**: Docusaurus site maintains Lighthouse scores of 90 or higher across Performance, Accessibility, Best Practices, and SEO.
**Constraints**:
- Constitution compliance (v1.0.0, e.g., technical accuracy, zero hallucination).
- Dark mode default and professional aesthetic.
- Exactly 3 co-learning elements per lesson (💡 Theory, 🎓 Key Insight, 💬 Ask your AI).
- 100% runnable code examples (ROS 2 Python, URDF, Gazebo plugins, Isaac Sim launches).
- Curriculum structure (Introduction, 4 Modules, Capstone).
- Content generation adheres to the content hierarchy.
- Use `curriculum.yaml` for sidebar auto-generation.
**Scale/Scope**: Generation of approx. 30-40 Markdown/MDX files for the complete textbook content.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

The plan strictly adheres to the Physical AI & Humanoid Robotics Textbook Constitution v1.0.0.
-   **Interdisciplinary Collaboration**: The content covers a broad range of topics fostering this.
-   **Ethical AI Development**: Explicitly includes a section on ethical considerations.
-   **Robustness & Safety Engineering**: Code snippets are runnable and verified.
-   **Human-Robot Interaction Design**: Ethical HRI considerations are included.
-   **Continuous Learning & Adaptation**: Emphasizes hands-on exercises and practical application.
-   **Technical Accuracy & Zero Hallucination**: Strict validation for all generated code and technical details.
-   **Co-Learning Pedagogy**: Enforces exactly 3 co-learning elements per lesson.

No violations detected; all gates passed.

## Project Structure

### Documentation (this feature)

```text
specs/0008-curriculum/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── checklists/
│   └── requirements.md  # Spec quality checklist
└── (other artifacts as generated)
```

### Source Code (repository root, specifically Docusaurus content in `frontend/`)

```text
frontend/
├── docs/
│   ├── introduction/          # Introduction chapters
│   │   ├── 01-foundations-of-physical-ai.md
│   │   ├── 02-from-digital-to-embodied.md
│   │   ├── 03-humanoid-robotics-landscape.md
│   │   ├── 04-sensor-systems.md
│   │   ├── 05-hardware-requirements.md
│   │   ├── 06-sim-to-real-tips.md
│   │   └── 07-ethical-hri-safety.md
│   ├── module-1-ros2/         # ROS 2 chapters
│   │   ├── 01-ros2-architecture.md
│   │   ├── 02-nodes-topics.md
│   │   ├── 03-services-actions.md
│   │   ├── 04-building-ros2-packages-python.md
│   │   ├── 05-launch-files-parameters.md
│   │   └── 06-urdf-for-humanoids.md
│   ├── module-2-digital-twin/ # Digital Twin chapters
│   │   ├── 01-gazebo-setup.md
│   │   ├── 02-urdf-sdf-formats.md
│   │   ├── 03-physics-sensor-simulation.md
│   │   └── 04-unity-visualization.md
│   ├── module-3-isaac/        # NVIDIA Isaac chapters
│   │   ├── 01-isaac-sdk-isaac-sim.md
│   │   ├── 02-ai-perception-manipulation.md
│   │   ├── 03-reinforcement-learning-robot-control.md
│   │   └── 04-sim-to-real-transfer.md
│   ├── module-4-vla/          # VLA chapters
│   │   ├── 01-humanoid-kinematics-dynamics.md
│   │   ├── 02-bipedal-locomotion-balance.md
│   │   ├── 03-manipulation-grasping.md
│   │   ├── 04-natural-human-robot-interaction.md
│   │   ├── 05-gpt-conversational-ai.md
│   │   ├── 06-speech-recognition-nlu.md
│   │   ├── 07-multi-modal-interaction.md
│   │   ├── 08-voice-to-action-whisper.md
│   │   └── 09-cognitive-planning-llms.md
│   └── capstone/              # Capstone chapters
│       ├── 01-capstone-project-overview.md
│       └── 02-capstone-rubric.md
├── curriculum.yaml           # Root-level curriculum definition for sidebar auto-generation
└── sidebars.ts               # Docusaurus sidebar configuration (will use curriculum.yaml to generate)
```

## Phase Breakdown

### Phase 1: Sidebar & Navigation Setup (Estimated: 30-45 min)

**Goal**: Configure Docusaurus to correctly use `curriculum.yaml` for dynamic sidebar generation and ensure all module categories are visible.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P1-T01  | Medium | `curriculum.yaml` | `frontend/docusaurus.config.ts`, `frontend/sidebars.ts` | `sidebars.ts` is configured to read `curriculum.yaml`; `docusaurus.config.ts` points to `sidebars.ts`; sidebar displays all module categories. |
| P1-T02  | Low    | P1-T01       | (None)       | Local Docusaurus build (`npm run start`) shows correct module categories in sidebar. |

### Phase 2: Introduction & Overview Pages (Estimated: 45-60 min)

**Goal**: Generate comprehensive content for the Introduction section, including new required chapters.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P2-T01  | Medium | `specs/0008-curriculum/spec.md` | `frontend/docs/introduction/01-foundations-of-physical-ai.md` | Chapter content generated; includes 3 co-learning elements. |
| P2-T02  | Medium | P2-T01       | `frontend/docs/introduction/02-from-digital-to-embodied.md` | Chapter content generated; includes 3 co-learning elements. |
| P2-T03  | Medium | P2-T02       | `frontend/docs/introduction/03-humanoid-robotics-landscape.md` | Chapter content generated; includes 3 co-learning elements. |
| P2-T04  | Medium | P2-T03       | `frontend/docs/introduction/04-sensor-systems.md` | Chapter content generated; includes 3 co-learning elements. |
| P2-T05  | Medium | P2-T04       | `frontend/docs/introduction/05-hardware-requirements.md` | Chapter content generated, including hardware requirements table; includes 3 co-learning elements. |
| P2-T06  | Medium | P2-T05       | `frontend/docs/introduction/06-sim-to-real-tips.md` | Chapter content generated, including sim-to-real tips; includes 3 co-learning elements. |
| P2-T07  | Medium | P2-T06       | `frontend/docs/introduction/07-ethical-hri-safety.md` | Chapter content generated, including ethical considerations in HRI; includes 3 co-learning elements. |

### Phase 3: Module 1-4 Lesson Generation (Estimated: 2-3 hours)

**Goal**: Generate all missing lesson content for Modules 1, 2, 3, and 4, ensuring co-learning elements and runnable code snippets. This phase can be parallelized by module.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P3-M1-T01 | Medium | `specs/0008-curriculum/spec.md` | `frontend/docs/module-1-ros2/01-ros2-architecture.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M1-T02 | Medium | P3-M1-T01    | `frontend/docs/module-1-ros2/02-nodes-topics.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M1-T03 | Medium | P3-M1-T02    | `frontend/docs/module-1-ros2/03-services-actions.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M1-T04 | Medium | P3-M1-T03    | `frontend/docs/module-1-ros2/04-building-ros2-packages-python.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M1-T05 | Medium | P3-M1-T04    | `frontend/docs/module-1-ros2/05-launch-files-parameters.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M1-T06 | Medium | P3-M1-T05    | `frontend/docs/module-1-ros2/06-urdf-for-humanoids.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M2-T01 | Medium | `specs/0008-curriculum/spec.md` | `frontend/docs/module-2-digital-twin/01-gazebo-setup.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M2-T02 | Medium | P3-M2-T01    | `frontend/docs/module-2-digital-twin/02-urdf-sdf-formats.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M2-T03 | Medium | P3-M2-T02    | `frontend/docs/module-2-digital-twin/03-physics-sensor-simulation.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M2-T04 | Medium | P3-M2-T03    | `frontend/docs/module-2-digital-twin/04-unity-visualization.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M3-T01 | Medium | `specs/0008-curriculum/spec.md` | `frontend/docs/module-3-isaac/01-isaac-sdk-isaac-sim.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M3-T02 | Medium | P3-M3-T01    | `frontend/docs/module-3-isaac/02-ai-perception-manipulation.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M3-T03 | Medium | P3-M3-T02    | `frontend/docs/module-3-isaac/03-reinforcement-learning-robot-control.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M3-T04 | Medium | P3-M3-T03    | `frontend/docs/module-3-isaac/04-sim-to-real-transfer.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T01 | Medium | `specs/0008-curriculum/spec.md` | `frontend/docs/module-4-vla/01-humanoid-kinematics-dynamics.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T02 | Medium | P3-M4-T01    | `frontend/docs/module-4-vla/02-bipedal-locomotion-balance.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T03 | Medium | P3-M4-T02    | `frontend/docs/module-4-vla/03-manipulation-grasping.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T04 | Medium | P3-M4-T03    | `frontend/docs/module-4-vla/04-natural-human-robot-interaction.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T05 | Medium | P3-M4-T04    | `frontend/docs/module-4-vla/05-gpt-conversational-ai.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T06 | Medium | P3-M4-T05    | `frontend/docs/module-4-vla/06-speech-recognition-nlu.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T07 | Medium | P3-M4-T06    | `frontend/docs/module-4-vla/07-multi-modal-interaction.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T08 | Medium | P3-M4-T07    | `frontend/docs/module-4-vla/08-voice-to-action-whisper.md` | Content generated, 3 co-learning elements, runnable code. |
| P3-M4-T09 | Medium | P3-M4-T08    | `frontend/docs/module-4-vla/09-cognitive-planning-llms.md` | Content generated, 3 co-learning elements, runnable code. |

### Phase 4: Capstone, Assessments, Hardware & Ethics Sections (Estimated: 45-60 min)

**Goal**: Generate Capstone content, including the rubric, and general assessment information.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P4-T01  | Medium | `specs/0008-curriculum/spec.md` | `frontend/docs/capstone/01-capstone-project-overview.md` | Content generated, 3 co-learning elements. |
| P4-T02  | Medium | P4-T01       | `frontend/docs/capstone/02-capstone-rubric.md` | Rubric content generated; includes 3 co-learning elements. |
| P4-T03  | Medium | `specs/0008-curriculum/spec.md` | `frontend/docs/assessments/01-assessments-overview.md` | Overview of assessments generated; includes 3 co-learning elements. |

### Phase 5: Verification & Polish (Estimated: 30-45 min)

**Goal**: Ensure the generated content is fully integrated, navigable, and meets quality standards.

| Task ID | Effort | Dependencies | Output Files | Acceptance Criteria |
|:--------|:-------|:-------------|:-------------|:--------------------|
| P5-T01  | Medium | All content tasks | (None)       | Local Docusaurus build (`npm run start` in `frontend/`) completes without errors. |
| P5-T02  | High   | P5-T01       | (None)       | All lesson pages accessible via sidebar; content displayed correctly. |
| P5-T03  | High   | P5-T02       | (None)       | Random sample of lessons confirms exactly 3 co-learning elements are present. |
| P5-T04  | High   | P5-T03       | (None)       | Random sample of code snippets are runnable (simulated verification). |
| P5-T05  | Medium | P5-T04       | (None)       | Sidebar navigation is correctly ordered and complete according to `curriculum.yaml`. |
| P5-T06  | Low    | P5-T05       | `frontend/docusaurus.config.ts` | Ensure `docusaurus.config.ts` correctly uses the new sidebar configuration. |

## Risk Assessment & Mitigations

| Risk                                     | Mitigation                                                                                                                              |
|:-----------------------------------------|:----------------------------------------------------------------------------------------------------------------------------------------|
| Long Content Generation Time             | Chunk generation into smaller, atomic tasks; leverage parallel processing capabilities (where agent supports).                         |
| Code Hallucination in Snippets           | Explicitly prompt for runnable code; perform simulated verification of generated snippets (P5-T04).                                    |
| Inconsistent Co-Learning Elements        | Strict adherence to the 3-element rule; automated check during verification (P5-T03).                                                   |
| Docusaurus Build Failures                | Incremental building and verification (P5-T01); careful update of `sidebars.ts` and `docusaurus.config.ts`.                             |
| Syllabus Misalignment                    | Cross-reference generated content against `specs/0008-curriculum/spec.md` and `curriculum.yaml`.                                      |
| Duplicate Content Generation             | Check for existing files before generating content for a chapter.                                                                        |
| Incorrect File Naming/Paths              | Adhere strictly to the `Content Hierarchy` section of the spec for file paths and numerical prefixes.                                  |

## Timeline Estimate (Hackathon-friendly)

-   **Phase 1 (Sidebar & Navigation Setup)**: 30-45 minutes
-   **Phase 2 (Introduction & Overview Pages)**: 45-60 minutes
-   **Phase 3 (Module 1-4 Lesson Generation)**: 2-3 hours
-   **Phase 4 (Capstone, Assessments, Hardware & Ethics Sections)**: 45-60 minutes
-   **Phase 5 (Verification & Polish)**: 30-45 minutes
-   **Total Estimated Agent Time**: ~5-6.5 hours

## Verification & Success Criteria

-   **Local Build Success**: `npm run start` and `npm run build` commands in `frontend/` must complete without errors.
-   **Co-Learning Elements**: All generated lesson pages must contain exactly one 💡 Theory, one 🎓 Key Insight, and one 💬 Ask your AI element.
-   **Sidebar Navigation**: The Docusaurus sidebar must correctly display the full curriculum hierarchy, allowing seamless navigation to all lessons.
-   **No Technical Hallucinations**: Code snippets and technical explanations must be accurate and runnable as verified through simulated execution or manual review.
-   **Constitution Adherence**: All generated content and structural changes must comply with the Physical AI & Humanoid Robotics Textbook Constitution v1.0.0.

## Next Steps
The project is now ready for content generation and integration.

**Next Recommended Command**: `/sp.implement` to execute the tasks outlined in this plan.
