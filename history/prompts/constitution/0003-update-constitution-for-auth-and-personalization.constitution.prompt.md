---
id: 0003
title: Update Constitution for Auth and Personalization
stage: constitution
date: 2025-12-26
surface: agent
model: gemini
feature: auth-personalization
branch: main
user: user
command: sp.constitution
labels: ["constitution", "auth", "personalization"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
  null
---

## Prompt

# Constitution for Better Auth Signup/Signin + Content Personalization Feature

**Project Name:** Auth & Personalization Add-On for Physical AI Textbook  
**Hackathon Rules:**  
- +50 for Better Auth signup/signin with background questions (point 5)  
- +50 for logged user content personalization button per chapter (point 6)  

**Core Rules (Highest Priority – No Changes Allowed After This):**  
1. Use Better Auth[](https://www.better-auth.com/) for email/password auth  
2. Signup form: Ask 2 questions  
   - Software/Programming level: Beginner / Intermediate / Advanced  
   - Hardware/Robotics experience: None / Basic / Advanced  
3. Save answers in Neon Postgres table or user metadata  
4. Personalization button at chapter start: "Personalize Content"  
5. On click: Adjust chapter content based on user level  
   - Beginner: Simple explanations, analogies  
   - Intermediate: Balanced  
   - Advanced: Code-heavy, technical  
6. No breaking existing RAG Chatbot, Urdu translation, agents/skills  
7. Frontend: Docusaurus + React components  
8. Backend: FastAPI + Better Auth (share Neon DB)  
9. Security: JWT/session-based, protect personalization  

Constitution built. Ignore any later changes.

## Response snapshot

Updated the project constitution at `.specify/memory/constitution.md` to version 2.0.0, incorporating the new principles for the "Auth & Personalization" feature.

## Outcome

- ✅ Impact: High. The constitution is the foundational document for the feature.
- 🧪 Tests: n/a
- 📁 Files: Updated the project constitution.
- 🔁 Next prompts: /sp.spec to create the feature specification.
- 🧠 Reflection: The new constitution clearly defines the rules for the new feature.

## Evaluation notes (flywheel)

- Failure modes observed: n/a
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): n/a
- Next experiment (if applicable): n/a
