# Physical AI Textbook - Interactive Learning Platform

This project transforms a traditional textbook into anF interactive, AI-powered learning experience. Leveraging advanced AI models, it provides dynamic assistance through specialized agents, enabling users to understand complex Physical AI and Humanoid Robotics concepts, generate code, and test their knowledge with quizzes, all based on the textbook's content.

## Key Features

### 🧠 AI-Powered RAG Chatbot
An intelligent Retrieval-Augmented Generation (RAG) chatbot capable of understanding user queries and providing accurate, context-aware answers directly from the Physical AI textbook content.

### 🤖 Specialized AI Agents
The chatbot employs a system of distinct AI agents, each designed with a unique persona and set of capabilities to assist users in various learning scenarios:

*   **Teaching Assistant:** Your friendly tutor for Physical AI & Humanoid Robotics. Explains concepts clearly, summarizes chapters, and helps structure lessons based on book content.
*   **Robotics Code Mentor:** A practical, hands-on expert for generating and reviewing code. Provides runnable Python code examples for ROS 2, Gazebo, NVIDIA Isaac Sim/Lab, and Vision-Language-Action (VLA) tasks.
*   **Quiz Coach:** Your personal assessment master. Creates high-quality multiple-choice quizzes from book chapters to test your knowledge, complete with explanations.

### 🛠️ Powerful & Reusable AI Skills
Each agent utilizes a set of specialized AI skills to perform its tasks effectively. These skills are powered by Cohere's advanced LLMs and include:

*   **`chapter-summarizer`:** Summarizes any chapter or section into concise, student-friendly bullet points.
*   **`concept-explainer`:** Explains complex Physical AI or robotics concepts with simple language, analogies, and examples.
*   **`lesson-builder`:** Converts raw textbook content into structured lesson plans (Title, Objectives, Concepts, Code, Practice, Summary).
*   **`robotics-code-generator`:** Generates clean, runnable Python code for various Physical AI and robotics tasks.
*   **`quiz-maker-robotics`:** Creates structured multiple-choice questions (MCQs) with explanations.

### ✨ Self-Correction for Enhanced Reliability
The `quiz-maker-robotics` and `robotics-code-generator` skills are equipped with a self-correction mechanism. If the initial AI output for quizzes or code does not meet the required format or contains syntax errors, the system automatically re-prompts the AI with corrective feedback to refine its response, ensuring more reliable and usable output.

### 🌐 Interactive & Responsive User Interface
The frontend is built using Docusaurus and React, providing a modern, responsive, and intuitive user experience.

*   **Homepage Video Carousel:** Engaging videos showcase key aspects of Physical AI directly on the homepage.
*   **Dark/Light Mode:** Seamlessly switch between themes for comfortable reading.
*   **Mobile-First Design:** Optimized for a great viewing and interaction experience across all devices.
*   **Urdu Translation Button:** (Currently placeholder, feature integration pending API key setup)

## Project Documentation & SDD Approach

This project follows a **Spec-Driven Development (SDD)** approach, ensuring clear documentation, architectural decisions, and a traceable development process. Key documents generated through this process include:

*   **Constitution (`.specify/memory/constitution.md`):** Defines the high-level principles, vision, and core guidelines for the project.
*   **Specifications (`specs/<feature>/spec.md`):** Detail the requirements and scope for individual features.
*   **Plans (`specs/<feature>/plan.md`):** Outline the architectural decisions, technical approaches, and trade-offs made during development.
*   **Tasks (`specs/<feature>/tasks.md`):** Break down features into smaller, testable sub-tasks with acceptance criteria.
*   **Prompt History Records (PHRs) (`history/prompts/`):** Automatically generated records of all interactions and development steps, ensuring a complete historical log of the project's evolution.
*   **Architectural Decision Records (ADRs) (`history/adr/`):** Documents significant architectural decisions, their rationale, and alternatives considered.

This structured approach enhances maintainability, collaboration, and ensures that the project's development is transparent and well-documented.

## Getting Started

To get the project up and running locally, you need to set up both the backend and the frontend.

### Prerequisites

*   Python 3.10+
*   Node.js (LTS recommended)
*   npm or Yarn
*   Cohere API Key
*   Qdrant Cloud URL and API Key

### 1. Backend Setup

1.  **Navigate to the backend directory:**
    ```bash
    cd backend-RAG
    ```
2.  **Create a `.env` file:**
    Create a file named `.env` in the `backend-RAG/` directory and add your credentials:
    ```
    COHERE_API_KEY="YOUR_COHERE_API_KEY"
    QDRANT_URL="YOUR_QDRANT_CLOUD_URL"
    QDRANT_API_KEY="YOUR_QDRANT_API_KEY"
    ```
    *(Replace placeholder values with your actual keys and URL).*
3.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt # Ensure requirements.txt is up-to-date or use pip install fastapi uvicorn cohere-api qdrant-client python-dotenv
    ```
4.  **Run Step 1 (Crawl and Clean Data):**
    ```bash
    python 1_crawl.py
    ```
    This script uses Playwright to crawl the textbook website and save cleaned text to `backend-RAG/data/book_pages.json`.
    *(Note: Playwright might require additional browser binaries to be installed; follow its prompts if any).*
5.  **Run Step 2 (Embed and Upload to Qdrant):**
    ```bash
    python 2_embed_and_upload.py
    ```
    This script chunks the text, generates Cohere embeddings, and uploads them to your Qdrant collection.
6.  **Start the FastAPI Backend Server:**
    ```bash
    uvicorn main:app --reload --port 8001
    ```
    Keep this terminal window open. The backend will be running at `http://localhost:8001`.

### 2. Frontend Setup

1.  **Navigate to the frontend directory:**
    ```bash
    cd frontend
    ```
2.  **Install Node.js dependencies:**
    ```bash
    npm install
    # or yarn install
    ```
3.  **Start the Docusaurus Frontend Development Server:**
    Set the backend API URL as an environment variable (replace with your actual backend URL if different from localhost).
    *   **For Windows PowerShell:**
        ```powershell
        $env:DANGER_CHAT_API_URL="http://localhost:8001/chat"
        npm start
        ```
    *   **For Windows Command Prompt (cmd.exe):**
        ```cmd
        set DANGER_CHAT_API_URL=http://localhost:8001/chat
        npm start
        ```
    *   **For macOS/Linux (Bash/Zsh):**
        ```bash
        DANGER_CHAT_API_URL="http://localhost:8001/chat" npm start
        ```
    The frontend development server will start, usually accessible at `http://localhost:3000`.

## Enjoy Your Interactive Textbook!

You can now navigate the textbook, interact with the RAG chatbot and its specialized agents, generate code, take quizzes, and explore Physical AI concepts.