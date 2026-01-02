# Research for Auth + Personalization

This document outlines research findings for the implementation of the authentication and personalization feature.

## Better Auth with FastAPI

**Decision**: Integrate Better Auth with FastAPI using the official `better-auth-fastapi` library.
**Rationale**: The library provides pre-built endpoints and utilities for common authentication tasks, reducing development time and ensuring adherence to best practices.
**Alternatives considered**: Building a custom authentication solution was rejected due to time constraints and the increased security risks.

## Neon DB with FastAPI

**Decision**: Use the `psycopg2-binary` library to connect to the Neon Postgres database from the FastAPI backend.
**Rationale**: `psycopg2` is a mature and widely-used library for connecting to Postgres from Python. The binary version simplifies installation.
**Alternatives considered**: `asyncpg` was considered for its asynchronous support, but `psycopg2` is sufficient for the current scope and is simpler to set up.

## Custom React Components in Docusaurus

**Decision**: Create custom React components for the `AuthButton` and `PersonalizeButton` and use Docusaurus's "swizzling" feature to integrate them into the theme. The `AuthButton` will be placed in the navbar, and the `PersonalizeButton` will be added to the top of each chapter page.
**Rationale**: Swizzling provides a clean way to override and extend Docusaurus theme components without maintaining a full fork of the theme.
**Alternatives considered**: Directly modifying the theme source code was rejected as it would make future Docusaurus updates difficult.

## State Management in Docusaurus

**Decision**: Use React's built-in Context API for managing the authentication state (e.g., current user, JWT).
**Rationale**: The authentication state is global and needs to be accessed by multiple components. The Context API is a lightweight solution that is well-suited for this purpose and avoids adding external dependencies like Redux or MobX.
**Alternatives considered**: Redux was considered but deemed overly complex for the current requirements.