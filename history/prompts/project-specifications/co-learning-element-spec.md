# Co-Learning Element Specification

## Context
A core principle of the "physical-ai-textbook" project, as established in `00-ADR.md`, is the inclusion of "exactly 3 Co-Learning elements per lesson." This document specifies the nature, types, and integration requirements for these interactive learning components. Co-Learning elements are designed to enhance engagement, reinforce concepts, and provide practical application opportunities within each chapter.

## Definition
A Co-Learning element is an interactive component or activity embedded within a textbook lesson, designed to facilitate active learning, self-assessment, or practical exploration. Each lesson *must* contain exactly three distinct Co-Learning elements.

## Core Principles
*   **Engagement:** Must actively involve the learner, moving beyond passive reading.
*   **Relevance:** Directly tied to the learning objectives of the lesson.
*   **Clarity:** Instructions and feedback must be clear and unambiguous.
*   **Independence:** Each element should ideally be self-contained but can contribute to a larger learning goal.
*   **Technical Feasibility:** Must be implementable within the Docusaurus frontend, potentially interacting with the FastAPI backend.

## Types of Co-Learning Elements (Examples)

While not limited to these, typical Co-Learning elements may include:

1.  **Interactive Quizzes/Checks for Understanding:**
    *   **Description:** Short, multiple-choice or fill-in-the-blank questions to test immediate comprehension.
    *   **Integration:** Can be static (client-side validation) or dynamic (submit answers to FastAPI for scoring/feedback).
    *   **UI/UX:** Clear question presentation, immediate feedback (correct/incorrect), optional explanations.
2.  **Code Snippet Execution / Parameter Tuning (using FastAPI):**
    *   **Description:** Allows learners to modify parameters of a code snippet or `curl` command and see immediate results from the backend (e.g., simulation output, calculation results).
    *   **Integration:** Form fields in Docusaurus frontend sending data to a FastAPI endpoint, displaying the response.
    *   **UI/UX:** Input fields for parameters, a "Run" button, and an output display area.
3.  **Visualizations / Interactive Diagrams:**
    *   **Description:** Dynamic graphs, charts, or diagrams where learners can manipulate variables and observe changes.
    *   **Integration:** Client-side JavaScript libraries (e.g., D3.js, Plotly.js) or visualizations rendered by the FastAPI backend (e.g., Matplotlib plots returned as images).
    *   **UI/UX:** Sliders, dropdowns, or direct manipulation elements to change visualization parameters.
4.  **Mini-Simulations / Concept Explorers:**
    *   **Description:** Small, focused simulations (e.g., a simple physics engine, a robot path planner) that illustrate a specific concept.
    *   **Integration:** Client-side (JavaScript-based) or backend-driven (FastAPI orchestrating a simple simulation and returning results/state).
    *   **UI/UX:** Canvas-based rendering, control panels, reset functionality.

## Integration with Docusaurus
*   Co-Learning elements will be implemented as custom React components within Docusaurus MDX files.
*   Components requiring backend interaction will utilize the FastAPI endpoints defined in `fastapi-backend-spec.md` and potentially chapter-specific API specs.

## Validation
*   Each lesson's content will be reviewed to ensure adherence to the "exactly 3 Co-Learning elements" rule.

## References
*   `00-ADR.md`
*   `docusaurus-platform-spec.md`
*   `fastapi-backend-spec.md`
