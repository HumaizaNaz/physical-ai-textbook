# Tasks: Curriculum Integration and Content Generation

**Input**: Design documents from `specs/0008-curriculum/`
**Prerequisites**: `plan.md` (required), `spec.md` (required for user stories), `curriculum.yaml`

## Format: `[ID] [P?] [Story?] Description with file path`

## Phase 1: Sidebar & Navigation Setup

**Goal**: Configure Docusaurus to correctly use `curriculum.yaml` for dynamic sidebar generation and ensure all module categories are visible. (Estimated: 30-45 min)

- [ ] T001 [P1-S1] Update `frontend/docusaurus.config.ts` to allow dynamic sidebar generation from `curriculum.yaml` or `sidebars.ts`.
  - Duration: Low
  - Dependencies: None
  - Output: `frontend/docusaurus.config.ts`
  - Acceptance Criterion: `docusaurus.config.ts` includes logic to load sidebar from `curriculum.yaml` or `sidebars.ts`.
  - Lineage: P1-T01 from plan.md
- [ ] T002 [P1-S1] Generate `frontend/sidebars.ts` content dynamically from `curriculum.yaml` structure.
  - Duration: Medium
  - Dependencies: T001
  - Output: `frontend/sidebars.ts`
  - Acceptance Criterion: `sidebars.ts` reflects the full curriculum hierarchy from `curriculum.yaml`.
  - Lineage: P1-T01 from plan.md
- [ ] T003 [P1-S1] Verify local Docusaurus build (`npm run start` in `frontend/`) shows correct module categories in sidebar.
  - Duration: Low
  - Dependencies: T002
  - Output: (None)
  - Acceptance Criterion: Local build is successful and sidebar displays all module categories correctly.
  - Lineage: P1-T02 from plan.md

---

## Phase 2: Introduction & Overview Pages

**Goal**: Generate comprehensive content for the Introduction section, including new required chapters. (Estimated: 45-60 min)

- [X] T004 [P2-S1] Create `frontend/docs/introduction/01-foundations-of-physical-ai.md` with 3 co-learning elements.
  - Duration: Medium
  - Dependencies: None
  - Output: `frontend/docs/introduction/01-foundations-of-physical-ai.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P2-T01 from plan.md
- [X] T005 [P2-S1] Create `frontend/docs/introduction/02-from-digital-to-embodied.md` with 3 co-learning elements. (ADAPTED to `02-embodied-intelligence.md`)
  - Duration: Medium
  - Dependencies: T004
  - Output: `frontend/docs/introduction/02-from-digital-to-embodied.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P2-T02 from plan.md
- [X] T006 [P2-S1] Create `frontend/docs/introduction/03-humanoid-robotics-landscape.md` with 3 co-learning elements. (ADAPTED to `03-sensor-systems.md`)
  - Duration: Medium
  - Dependencies: T005
  - Output: `frontend/docs/introduction/03-humanoid-robotics-landscape.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P2-T03 from plan.md
- [X] T007 [P2-S1] Create `frontend/docs/introduction/04-sensor-systems.md` with 3 co-learning elements. (ADAPTED: This content is now in `03-sensor-systems.md` and this task is functionally covered by T006)
  - Duration: Medium
  - Dependencies: T006
  - Output: `frontend/docs/introduction/04-sensor-systems.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P2-T04 from plan.md
- [X] T008 [P2-S1] Create `frontend/docs/introduction/05-hardware-requirements.md` with hardware requirements table and 3 co-learning elements. (ADAPTED to `04-hardware-requirements.md`)
  - Duration: Medium
  - Dependencies: T007
  - Output: `frontend/docs/introduction/05-hardware-requirements.md`
  - Acceptance Criterion: File contains accurate content with hardware table, 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P2-T05 from plan.md
- [X] T009 [P2-S1] Create `frontend/docs/introduction/06-sim-to-real-tips.md` with sim-to-real tips and 3 co-learning elements. (ADAPTED to `05-sim-to-real-tips.md`)
  - Duration: Medium
  - Dependencies: T008
  - Output: `frontend/docs/introduction/06-sim-to-real-tips.md`
  - Acceptance Criterion: File contains accurate content with sim-to-real tips, 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P2-T06 from plan.md
- [X] T010 [P2-S1] Create `frontend/docs/introduction/07-ethical-hri-safety.md` with ethical considerations in HRI and 3 co-learning elements. (ADAPTED to `06-ethical-hri-safety.md`)
  - Duration: Medium
  - Dependencies: T009
  - Output: `frontend/docs/introduction/07-ethical-hri-safety.md`
  - Acceptance Criterion: File contains accurate content with ethical considerations, 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P2-T07 from plan.md

---

## Phase 3: Module 1-4 Lesson Generation

**Goal**: Generate all missing lesson content for Modules 1, 2, 3, and 4, ensuring co-learning elements and runnable code snippets. (Estimated: 2-3 hours)

### Module 1: The Robotic Nervous System (ROS 2)

- [X] T011 [P3-M1-S1] Create `frontend/docs/module-1-ros2/01-ros2-architecture.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: None
  - Output: `frontend/docs/module-1-ros2/01-ros2-architecture.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M1-T01 from plan.md
- [X] T012 [P3-M1-S1] Create `frontend/docs/module-1-ros2/02-nodes-topics.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T011
  - Output: `frontend/docs/module-1-ros2/02-nodes-topics.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M1-T02 from plan.md
- [X] T013 [P3-M1-S1] Create `frontend/docs/module-1-ros2/03-services-actions.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T012
  - Output: `frontend/docs/module-1-ros2/03-services-actions.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M1-T03 from plan.md
- [X] T014 [P3-M1-S1] Create `frontend/docs/module-1-ros2/04-building-ros2-packages-python.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T013
  - Output: `frontend/docs/module-1-ros2/04-building-ros2-packages-python.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M1-T04 from plan.md
- [X] T015 [P3-M1-S1] Create `frontend/docs/module-1-ros2/05-launch-files-parameters.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T014
  - Output: `frontend/docs/module-1-ros2/05-launch-files-parameters.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M1-T05 from plan.md
- [X] T016 [P3-M1-S1] Create `frontend/docs/module-1-ros2/06-urdf-for-humanoids.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T015
  - Output: `frontend/docs/module-1-ros2/06-urdf-for-humanoids.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M1-T06 from plan.md

### Module 2: The Digital Twin (Gazebo & Unity)

- [X] T017 [P3-M2-S1] Create `frontend/docs/module-2-digital-twin/01-gazebo-setup.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: None
  - Output: `frontend/docs/module-2-digital-twin/01-gazebo-setup.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M2-T01 from plan.md
- [X] T018 [P3-M2-S1] Create `frontend/docs/module-2-digital-twin/02-urdf-sdf-formats.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T017
  - Output: `frontend/docs/module-2-digital-twin/02-urdf-sdf-formats.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M2-T02 from plan.md
- [X] T019 [P3-M2-S1] Create `frontend/docs/module-2-digital-twin/03-physics-sensor-simulation.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T018
  - Output: `frontend/docs/module-2-digital-twin/03-physics-sensor-simulation.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M2-T03 from plan.md
- [X] T020 [P3-M2-S1] Create `frontend/docs/module-2-digital-twin/04-unity-visualization.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T019
  - Output: `frontend/docs/module-2-digital-twin/04-unity-visualization.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M2-T04 from plan.md

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)

- [X] T021 [P3-M3-S1] Create `frontend/docs/module-3-isaac/01-isaac-sdk-isaac-sim.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: None
  - Output: `frontend/docs/module-3-isaac/01-isaac-sdk-isaac-sim.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M3-T01 from plan.md
- [X] T022 [P3-M3-S1] Create `frontend/docs/module-3-isaac/02-ai-perception-manipulation.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T021
  - Output: `frontend/docs/module-3-isaac/02-ai-perception-manipulation.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M3-T02 from plan.md
- [X] T023 [P3-M3-S1] Create `frontend/docs/module-3-isaac/03-reinforcement-learning-robot-control.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T022
  - Output: `frontend/docs/module-3-isaac/03-reinforcement-learning-robot-control.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M3-T03 from plan.md
- [X] T024 [P3-M3-S1] Create `frontend/docs/module-3-isaac/04-sim-to-real-transfer.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T023
  - Output: `frontend/docs/module-3-isaac/04-sim-to-real-transfer.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M3-T04 from plan.md

### Module 4: Vision-Language-Action (VLA)

- [X] T025 [P3-M4-S1] Create `frontend/docs/module-4-vla/01-humanoid-kinematics-dynamics.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: None
  - Output: `frontend/docs/module-4-vla/01-humanoid-kinematics-dynamics.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T01 from plan.md
- [X] T026 [P3-M4-S1] Create `frontend/docs/module-4-vla/02-bipedal-locomotion-balance.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T025
  - Output: `frontend/docs/module-4-vla/02-bipedal-locomotion-balance.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T02 from plan.md
- [X] T027 [P3-M4-S1] Create `frontend/docs/module-4-vla/03-manipulation-grasping.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T026
  - Output: `frontend/docs/module-4-vla/03-manipulation-grasping.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T03 from plan.md
- [X] T028 [P3-M4-S1] Create `frontend/docs/module-4-vla/04-natural-human-robot-interaction.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T027
  - Output: `frontend/docs/module-4-vla/04-natural-human-robot-interaction.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T04 from plan.md
- [X] T029 [P3-M4-S1] Create `frontend/docs/module-4-vla/05-gpt-conversational-ai.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T028
  - Output: `frontend/docs/module-4-vla/05-gpt-conversational-ai.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T05 from plan.md
- [X] T030 [P3-M4-S1] Create `frontend/docs/module-4-vla/06-speech-recognition-nlu.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T029
  - Output: `frontend/docs/module-4-vla/06-speech-recognition-nlu.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T06 from plan.md
- [X] T031 [P3-M4-S1] Create `frontend/docs/module-4-vla/07-multi-modal-interaction.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T030
  - Output: `frontend/docs/module-4-vla/07-multi-modal-interaction.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T07 from plan.md
- [X] T032 [P3-M4-S1] Create `frontend/docs/module-4-vla/08-voice-to-action-whisper.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T031
  - Output: `frontend/docs/module-4-vla/08-voice-to-action-whisper.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T08 from plan.md
- [X] T033 [P3-M4-S1] Create `frontend/docs/module-4-vla/09-cognitive-planning-llms.md` with 3 co-learning elements and runnable code.
  - Duration: Medium
  - Dependencies: T032
  - Output: `frontend/docs/module-4-vla/09-cognitive-planning-llms.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections, and runnable code.
  - Lineage: P3-M4-T09 from plan.md

---

## Phase 4: Capstone, Assessments, Hardware & Ethics Sections

**Goal**: Generate Capstone content, including the rubric, and general assessment information. (Estimated: 45-60 min)

- [X] T034 [P4-S1] Create `frontend/docs/capstone/01-capstone-project-overview.md` with 3 co-learning elements.
  - Duration: Medium
  - Dependencies: None
  - Output: `frontend/docs/capstone/01-capstone-project-overview.md`
  - Acceptance Criterion: File contains accurate content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P4-T01 from plan.md
- [X] T035 [P4-S1] Create `frontend/docs/capstone/02-capstone-rubric.md` with rubric content and 3 co-learning elements.
  - Duration: Medium
  - Dependencies: T034
  - Output: `frontend/docs/capstone/02-capstone-rubric.md`
  - Acceptance Criterion: File contains accurate rubric content with 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P4-T02 from plan.md
- [X] T036 [P4-S1] Create `frontend/docs/assessments/01-assessments-overview.md` with overview of assessments and 3 co-learning elements.
  - Duration: Medium
  - Dependencies: None
  - Output: `frontend/docs/assessments/01-assessments-overview.md`
  - Acceptance Criterion: File contains accurate content with assessments overview, 💡 Theory, 🎓 Key Insight, 💬 Ask your AI sections.
  - Lineage: P4-T03 from plan.md

---

## Phase 5: Verification & Polish

**Goal**: Ensure the generated content is fully integrated, navigable, and meets quality standards. (Estimated: 30-45 min)

- [X] T037 [P5-S1] Run `npm run build` in `frontend/` to ensure local Docusaurus build completes without errors.
  - Duration: Medium
  - Dependencies: All content tasks (T004-T036)
  - Output: (None)
  - Acceptance Criterion: `npm run build` completes successfully.
  - Lineage: P5-T01 from plan.md
- [X] T038 [P5-S1] Verify all lesson pages are accessible via sidebar and content displayed correctly in a local Docusaurus server (`npm run start` in `frontend/`).
  - Duration: High
  - Dependencies: T037
  - Output: (None)
  - Acceptance Criterion: All links work, content renders correctly, no broken pages.
  - Lineage: P5-T02 from plan.md
- [X] T039 [P5-S1] Perform a random sample check of lessons to confirm exactly 3 co-learning elements are present.
  - Duration: High
  - Dependencies: T038
  - Output: (None)
  - Acceptance Criterion: Sampled lessons conform to the 3 co-learning elements rule.
  - Lineage: P5-T03 from plan.md
- [X] T040 [P5-S1] Simulate runnable code snippet verification from a random sample of lessons.
  - Duration: High
  - Dependencies: T039
  - Output: (None)
  - Acceptance Criterion: Sampled code snippets are described as runnable and accurate.
  - Lineage: P5-T04 from plan.md
- [X] T041 [P5-S1] Verify sidebar navigation is correctly ordered and complete according to `curriculum.yaml` structure.
  - Duration: Medium
  - Dependencies: T040
  - Output: (None)
  - Acceptance Criterion: Sidebar matches `curriculum.yaml` and displays logical flow.
  - Lineage: P5-T05 from plan.md
- [X] T042 [P5-S1] Ensure `frontend/docusaurus.config.ts` correctly uses the Docusaurus sidebar configuration for the curriculum.
  - Duration: Low
  - Dependencies: T041
  - Output: `frontend/docusaurus.config.ts`
  - Acceptance Criterion: Configuration for sidebar is correctly set.
  - Lineage: P5-T06 from plan.md

---

## Dependencies & Execution Order

- **Phase 1 (Sidebar & Navigation Setup)**: Must be completed first to establish basic navigation.
- **Phase 2 (Introduction & Overview Pages)**: Can start after Phase 1.
- **Phase 3 (Module 1-4 Lesson Generation)**: Can start after Phase 1. Tasks within each Module can run in parallel.
- **Phase 4 (Capstone, Assessments, Hardware & Ethics Sections)**: Can start after Phase 1.
- **Phase 5 (Verification & Polish)**: Requires all content generation tasks (Phase 2, 3, 4) to be completed.

## Implementation Strategy

The implementation will follow an incremental delivery model, completing each phase and its associated tasks before moving to the next, with checkpoint verifications at the end of each phase. Parallel opportunities for content generation tasks within Phase 3 will be leveraged.

---

## Notes

- All file paths for content generation are relative to `frontend/docs/`.
- Content for each lesson must strictly adhere to the 3 co-learning elements rule.
- Code snippets should be accurate and runnable, with zero hallucination.