# Deployment Plan

## Context
This Deployment Plan outlines the strategy and procedures for deploying the "physical-ai-textbook" project, encompassing both the Docusaurus frontend and the FastAPI backend components. The goal is to ensure a reliable, scalable, and maintainable deployment process.

## Objectives
*   Ensure the Docusaurus frontend is publicly accessible and performant.
*   Ensure the FastAPI backend is publicly accessible, scalable, and secure.
*   Establish a consistent and automated (where possible) deployment process.
*   Minimize downtime during updates and new releases.
*   Provide clear rollback procedures in case of deployment failures.

## Deployment Architecture
### Frontend (Docusaurus)
*   **Hosting Platform:** Docusaurus is typically deployed as static assets.
    *   **Option 1 (Recommended):** Vercel, Netlify, GitHub Pages, or similar static site hosting services. These platforms provide excellent performance, CDN distribution, and often automated deployments from Git repositories.
    *   **Option 2 (Self-Hosted):** Serve static assets via a web server (e.g., Nginx, Apache) on a cloud VM.
*   **Build Process:** Docusaurus's build command (`npm run build`) generates static HTML, CSS, and JavaScript files.

### Backend (FastAPI)
*   **Hosting Platform:** Requires a server environment capable of running Python applications.
    *   **Option 1 (Recommended):** Platform as a Service (PaaS) like Render, Heroku, Google App Engine, AWS Elastic Beanstalk. These provide managed services, scaling, and simplified deployment.
    *   **Option 2 (Containerization):** Deploy using Docker containers on Kubernetes (GKE, EKS, AKS) or a container service (AWS Fargate, Google Cloud Run). This offers excellent scalability and environment consistency.
    *   **Option 3 (Virtual Machine):** Deploy on a cloud VM (e.g., AWS EC2, Google Compute Engine, Azure VM) with a reverse proxy (Nginx/Gunicorn) and process manager (systemd, Supervisor).

## Deployment Process
### 1. Version Control
*   All code (frontend and backend) will be managed in a Git repository (e.g., GitHub).
*   Deployment will be triggered from specific branches (e.g., `main` or `production`).

### 2. Continuous Integration (CI)
*   **Purpose:** Automate testing and build processes upon code commits.
*   **Tools:** GitHub Actions, GitLab CI/CD, Jenkins.
*   **Steps:**
    *   Run linting and formatting checks.
    *   Execute unit and integration tests (for both frontend and backend).
    *   Build the Docusaurus frontend.
    *   Build Docker images for the FastAPI backend (if containerized deployment is chosen).

### 3. Continuous Deployment (CD)
*   **Purpose:** Automate deployment to staging and production environments.
*   **Tools:** Integrated into chosen hosting platforms (e.g., Vercel's Git integration, GitHub Actions for deploying to PaaS/VMs).

#### Deployment to Staging
*   Automatically deploy successful CI builds from a `develop` or `staging` branch to a staging environment.
*   **Verification:** Conduct E2E tests and manual QA on the staging environment.

#### Deployment to Production
*   Triggered manually or automatically after successful staging verification, usually from the `main` or `production` branch.
*   **Zero-Downtime Deployment:** Implement strategies like blue/green deployment or rolling updates to ensure continuous service availability.
*   **Monitoring:** Verify application health and performance post-deployment.

## Rollback Strategy
*   In case of critical issues post-deployment, immediately roll back to the previous stable version.
*   Hosting platforms often provide built-in rollback capabilities. For containerized deployments, revert to the previous Docker image.

## Environmental Variables & Secrets Management
*   Sensitive information (API keys, database credentials) will be managed using environment variables (e.g., GitHub Secrets, cloud provider secret management services) and *not* committed to the repository.

## Monitoring & Logging
*   **Frontend:** Browser developer tools, analytics platforms (e.g., Google Analytics).
*   **Backend:** Cloud provider logging services (e.g., Google Cloud Logging, AWS CloudWatch), application performance monitoring (APM) tools.

## Domain & SSL
*   Configure custom domains and ensure SSL certificates are in place for secure communication (HTTPS).

## References
*   `00-ADR.md`
*   `00-PLAN.md`
*   `test-plan.md`
*   `docusaurus-platform-spec.md`
*   `fastapi-backend-spec.md`
