---
sidebar_position: 2
title: Capstone Project Rubric
---

# Capstone Project Rubric: Evaluating Your Autonomous Humanoid

The Capstone Project is a comprehensive assessment of your ability to integrate knowledge and skills across all modules of this textbook. This rubric outlines the criteria for evaluation, providing clear expectations for each component of your autonomous simulated humanoid.

## Project Evaluation Categories

Your Capstone Project will be assessed across several key categories, each contributing to the overall score.

### 1. System Design and Architecture (25%)

*   **Criteria**:
    *   **Modularity**: Clear separation of concerns into distinct ROS 2 nodes/packages.
    *   **Scalability**: Design considerations for expanding capabilities or operating in more complex environments.
    *   **Robustness**: Handling of expected failures or ambiguities (e.g., in voice commands, object detection).
    *   **Documentation**: Clarity and completeness of internal documentation (code comments, architecture diagrams, `README.md` files).
*   **Exemplary (A)**: Highly modular, scalable architecture. Robust error handling demonstrated. Excellent, clear documentation.
*   **Proficient (B)**: Well-designed, mostly modular architecture. Basic error handling. Good documentation.
*   **Developing (C)**: Functional but with some architectural flaws or lack of modularity. Limited error handling. Adequate documentation.
*   **Needs Improvement (D/F)**: Poorly organized design, significant architectural issues, or absent documentation.

### 2. Implementation and Technical Execution (30%)

*   **Criteria**:
    *   **Functionality**: All core features (voice command interpretation, perception, navigation, manipulation) are implemented and functional.
    *   **Code Quality**: Clean, readable, well-structured, and efficient code (Python, launch files, etc.).
    *   **Technical Accuracy**: Correct use of ROS 2 concepts (nodes, topics, services, actions), Isaac Sim/ROS APIs, and other tools.
    *   **Reproducibility**: Project can be easily set up and run by others.
*   **Exemplary (A)**: All features fully functional. Exceptionally high code quality. Impeccable technical accuracy. Easy reproducibility.
*   **Proficient (B)**: All core features functional. Good code quality. Correct technical implementation. Reproducible.
*   **Developing (C)**: Core features mostly functional, but with minor bugs or limitations. Average code quality. Minor technical inaccuracies. Reproducibility might require assistance.
*   **Needs Improvement (D/F)**: Significant non-functional features, poor code quality, or numerous technical errors. Difficult to reproduce.

### 3. Voice Command Interpretation and Planning (20%)

*   **Criteria**:
    *   **NLU Effectiveness**: Accuracy and robustness of translating natural language commands into robot-executable plans.
    *   **Task Decomposition**: Ability to break down complex commands into appropriate sub-tasks.
    *   **LLM Integration**: Effective use of LLMs (or other NLU components) for cognitive planning.
*   **Exemplary (A)**: Interprets complex commands robustly. Generates optimal, flexible task plans. Demonstrates advanced LLM integration.
*   **Proficient (B)**: Interprets most commands. Generates logical task plans. Effective LLM integration.
*   **Developing (C)**: Interprets simple commands with some errors. Basic task planning. Limited LLM integration.
*   **Needs Improvement (D/F)**: Fails to interpret commands or generate coherent plans.

### 4. Robot Autonomy and Task Execution (25%)

*   **Criteria**:
    *   **Autonomous Navigation**: Robot moves to target locations, avoids obstacles (using Nav2).
    *   **Perception Accuracy**: Correctly identifies and localizes objects/targets.
    *   **Manipulation Proficiency**: Successfully grasps and interacts with objects.
    *   **Robustness to Uncertainty**: Handles minor variations in environment or object properties.
    *   **End-to-End Performance**: Seamless execution of the complete task sequence.
*   **Exemplary (A)**: Flawless autonomous execution of complex tasks. Highly robust perception and manipulation.
*   **Proficient (B)**: Autonomous execution of most tasks with minor hiccups. Good perception and manipulation.
*   **Developing (C)**: Executes simple tasks but struggles with complexity or errors. Basic perception and manipulation.
*   **Needs Improvement (D/F)**: Fails to execute tasks autonomously or consistently.

---

### Co-Learning Elements

#### 💡 Theory: Iterative Design & Evaluation
Project-based learning, especially capstone projects, highlights the iterative nature of design and evaluation in engineering. You don't build a perfect system on the first try. Instead, you design, implement, test, identify flaws, refine, and repeat. This iterative cycle is fundamental to robust system development.

#### 🎓 Key Insight: The Integration Challenge
The true challenge of a capstone project often lies not in mastering individual components, but in their seamless integration. Making diverse modules (perception, planning, control, NLP) communicate and operate coherently as a single system is where most real-world engineering difficulties arise. It tests your understanding of system architecture.

#### 💬 Practice Exercise: Ask your AI

**Prompt**: "You are grading a Capstone project for a humanoid robot. A student's robot successfully navigates to objects but frequently fails to grasp them due to imprecise arm movements. Propose a specific feedback statement and a suggestion for improvement based on the rubric categories (e.g., Implementation, Robot Autonomy)."

**Instructions**: Use your preferred AI assistant to:
1.  Formulate a constructive feedback statement for the student.
2.  Suggest specific areas for improvement, linking back to the rubric (e.g., focusing on improving manipulation proficiency under "Robot Autonomy and Task Execution" by refining control or perception).
```