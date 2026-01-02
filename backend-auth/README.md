---
title: Better Auth Backend
emoji: 🚀
colorFrom: purple
colorTo: yellow
sdk: docker
pinned: false
license: apache-2.0
---

# Better Auth Backend

This is a Node.js/TypeScript backend authentication service built with Hono, Express, and better-auth. It provides user authentication, registration, and profile management functionality.

## Deployment to Hugging Face Spaces

This project is configured for deployment to Hugging Face Spaces using Docker.

### Files included:
- `Dockerfile` - Container configuration for deployment
- `server.ts` - Main application server using Hono framework
- `package.json` - Dependencies and scripts

### Environment Variables:
The application requires the following environment variables to be set:
- `DATABASE_URL` - PostgreSQL database connection string
- `AUTH_SECRET` - Secret for JWT token signing
- `FRONTEND_URL` - Allowed origin for CORS (default: http://localhost:3000)

### Port Configuration:
The application listens on port 7860 as required by Hugging Face Spaces.

## Local Development

To run this application locally:

1. Install dependencies: `npm install`
2. Set up environment variables in a `.env` file
3. Run in development mode: `npm run dev`
4. Build for production: `npm run build`
5. Start the production server: `npm start`"# better-auth" 
