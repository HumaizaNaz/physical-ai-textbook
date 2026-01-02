# Quickstart for Auth + Personalization

This guide provides the steps to set up and run the authentication and personalization feature locally.

## Prerequisites

- Python 3.11+
- Node.js 18+
- A running Neon Postgres database

## Backend Setup

1.  **Navigate to the backend directory:**
    ```bash
    cd backend
    ```

2.  **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

3.  **Configure environment variables:**
    Create a `.env` file in the `backend` directory with the following content:
    ```
    DATABASE_URL="your_neon_database_url"
    BETTER_AUTH_SECRET="your_better_auth_secret"
    ```

4.  **Run the backend server:**
    ```bash
    uvicorn main:app --reload
    ```
    The backend will be running at `http://127.0.0.1:8000`.

## Frontend Setup

1.  **Navigate to the frontend directory:**
    ```bash
    cd frontend
    ```

2.  **Install dependencies:**
    ```bash
    npm install
    ```

3.  **Run the frontend development server:**
    ```bash
    npm start
    ```
    The frontend will be running at `http://localhost:3000`.

## Running the Feature

1.  Open your browser and navigate to `http://localhost:3000`.
2.  Click the "Sign Up" button and create a new account, selecting your experience levels.
3.  Log in with your new account.
4.  Navigate to any chapter and click the "Personalize Content" button to see the content change.