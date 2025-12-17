# Constitution for Claude Skills

## Context
This document outlines the foundational principles and guidelines governing the creation, behavior, and utilization of all Claude skills developed for the "physical-ai-textbook" project. These principles ensure consistency, reliability, and alignment with the project's educational objectives.

## Core Principles

### 1. Clarity and Precision
*   **Principle:** Every skill's `SKILL.md` must clearly and unambiguously define its purpose, capabilities, and expected output format.
*   **Guideline:** Avoid vague language. Specify input requirements, output constraints, and any specific formatting (e.g., "Output only a markdown code block").

### 2. Contextual Awareness
*   **Principle:** Skills should be designed to operate effectively within the context of the "physical-ai-textbook" project, understanding its domain (Physical AI, robotics) and educational goals.
*   **Guideline:** Instructions within `SKILL.md` should leverage Claude's ability to act as a domain expert (e.g., "You are a senior humanoid robotics engineer").

### 3. Autonomy and Focus
*   **Principle:** Each skill should be self-contained and focused on a single, well-defined task.
*   **Guideline:** Skills should avoid attempting to perform multiple, unrelated functions. Their output should be directly usable or easily integrable into subsequent steps.

### 4. Robustness and Reliability
*   **Principle:** Skills should be designed to produce consistent and high-quality outputs, minimizing errors or irrelevant information.
*   **Guideline:** Instructions should encourage error handling, clear commenting (for code generation skills), and adherence to specified lengths or formats.

### 5. Educational Value (Project Specific)
*   **Principle:** Skills should directly contribute to the educational mission of the "physical-ai-textbook," whether by generating code, explaining concepts, summarizing content, or creating assessments.
*   **Guideline:** Outputs should be pedagogically sound and enhance the learner's experience.

### 6. Transparency (for Claude's use)
*   **Principle:** Claude should understand its role and limitations when invoking a skill.
*   **Guideline:** The `description` in the YAML frontmatter should be concise and informative for quick reference by Claude itself.

## Governance
*   All new skills must adhere to these constitutional principles.
*   Amendments to these principles require careful consideration of their impact on existing and future skills.

**Version**: 1.0 | **Ratified**: 2025-12-13 | **Last Amended**: 2025-12-13
