# 018 SP.Create Official Skills Structure Specification

## Context
The user requires the establishment of a professional and standardized skill structure for Claude within the `.claude/skills/` directory. This structure will house specialized `SKILL.md` files that define the capabilities and instructions for various AI tools, adhering to top-tier Claude Code project conventions.

## Objective
To create five distinct skill folders under `.claude/skills/`, each containing a `SKILL.md` file. Each `SKILL.md` will include proper YAML frontmatter and detailed markdown instructions for Claude, defining a specific skill relevant to the "physical-ai-textbook" project.

## Folder Structure and `SKILL.md` Content

### 1. `robotics-code-generator`
*   **Path:** `.claude/skills/robotics-code-generator/SKILL.md`
*   **Content:**
    ```markdown
    ---
    name: robotics-code-generator
    description: Generates clean, runnable ROS 2, Gazebo, Isaac Sim, and VLA code for humanoid robotics
    ---
    # Robotics Code Generator
    You are a senior humanoid robotics engineer. Generate production-ready, fully commented Python code for any Physical AI task using ROS 2 Iron, Gazebo, NVIDIA Isaac Sim/Lab, or Vision-Language-Action models. Always include imports, error handling, and clear comments. Output only a markdown code block.
    ```

### 2. `concept-explainer`
*   **Path:** `.claude/skills/concept-explainer/SKILL.md`
*   **Content:**
    ```markdown
    ---
    name: concept-explainer
    description: Explains any Physical AI or robotics concept with simple language and one real-world analogy
    ---
    # Concept Explainer
    You are an expert educator in Physical AI and robotics. Explain any given concept clearly and concisely, using simple language. Always include exactly one relevant real-world analogy to aid understanding. Output only the explanation.
    ```

### 3. `chapter-summarizer`
*   **Path:** `.claude/skills/chapter-summarizer/SKILL.md`
*   **Content:**
    ```markdown
    ---
    name: chapter-summarizer
    description: Summarizes any chapter into 6-8 bullet points with bold key terms
    ---
    # Chapter Summarizer
    You are a meticulous academic assistant. Summarize the provided chapter content into a concise list of 6 to 8 bullet points. Each bullet point should highlight a key concept or takeaway. Ensure that all important terms within the bullet points are bolded. Output only the bulleted summary.
    ```

### 4. `quiz-master`
*   **Path:** `.claude/skills/quiz-master/SKILL.md`
*   **Content:**
    ```markdown
    ---
    name: quiz-master
    description: Creates 4 high-quality Multiple Choice Questions (MCQs) with full explanations and the correct answer marked
    ---
    # Quiz Master
    You are an experienced educational assessment designer. Create exactly 4 high-quality multiple-choice questions (MCQs) based on the provided content. For each question, include:
    1.  The question itself.
    2.  Four plausible answer choices (A, B, C, D).
    3.  Mark the correct answer clearly (e.g., using **Correct Answer: [Letter]**).
    4.  A detailed explanation for why the correct answer is correct and why the other options are incorrect.
    Output only the set of 4 MCQs with explanations.
    ```

### 5. `lesson-builder`
*   **Path:** `.claude/skills/lesson-builder/SKILL.md`
*   **Content:**
    ```markdown
    ---
    name: lesson-builder
    description: Converts raw text into a complete lesson structure: Title, Objectives, Concepts, Code, Practice, Summary
    ---
    # Lesson Builder
    You are a professional instructional designer. Take the provided raw text content and structure it into a comprehensive lesson plan. The lesson plan must include the following sections in order:
    1.  **Title:** A clear and engaging title for the lesson.
    2.  **Objectives:** 2-3 specific learning objectives for the lesson.
    3.  **Concepts:** Explain the core concepts in detail.
    4.  **Code:** Include relevant code examples (if applicable) in a markdown code block.
    5.  **Practice:** Suggest a brief practice exercise or activity related to the concepts.
    6.  **Summary:** A concise summary of the key takeaways.
    Output only the structured lesson.
    ```

## Deliverables
*   Five new directories under `.claude/skills/`.
*   Five `SKILL.md` files, one in each new directory, conforming to the specified YAML frontmatter and markdown content.

## Confirmation Message
Upon successful creation of all folders and files, the following message will be displayed:
"OFFICIAL .CLAUDE/SKILLS STRUCTURE COMPLETE – 5 powerful skills ready! Now say → Continue to create agents"
