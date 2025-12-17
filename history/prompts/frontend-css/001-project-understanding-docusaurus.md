# 001 - Project Understanding: Physical AI Textbook (Docusaurus)

## Context

The user requested understanding of their project, emphasizing it is a "book" and not the previously discussed "bank" system. This led to a focused investigation of project definition files.

## Actions Taken

1. **Read `README.md`**: Found to be very brief, indicating a need for deeper investigation.
2. **Read `.specify/memory/constitution.md`**: Identified as a template, providing structural insight but no specific project content.
3. **Read `frontend/docs/00-ADR.md`**:
    * Confirmed key architectural decisions: FastAPI backend for live demos/grading, Docusaurus frontend.
    * Established core constraints: 3 Co-Learning elements per lesson, 1 live curl per chapter, simulation-only, dark mode default.
4. **Read `frontend/docs/00-PLAN.md`**:
    * Outlined the chapter structure and development timeline (Intro, ROS 2, Gazebo + Unity, NVIDIA Isaac Sim, VLA + Capstone).
    * Reiterated locked rules for chapter content.

## Project Understanding

The project is identified as a "physical-ai-textbook". It utilizes a Docusaurus frontend for content delivery and a FastAPI backend for interactive elements (live demos, grading, curl commands). The content is structured as an educational textbook, focusing on various aspects of physical AI and adhering to specific content and technical constraints.

## Outcome

A clear understanding of the project's purpose, architecture, and constraints was established. The focus shifted entirely to the "physical-ai-textbook" context, confirming the user's emphasis on it being a "book."
