# Data Model for Auth + Personalization

This document defines the data models for the authentication and personalization feature.

## User

Represents a person interacting with the application.

**Fields**:
- `id`: UUID, Primary Key
- `email`: TEXT, UNIQUE, NOT NULL
- `hashed_password`: TEXT, NOT NULL
- `created_at`: TIMESTAMP, default NOW()

## UserBackground

Stores the background information for a user.

**Fields**:
- `user_id`: UUID, Foreign Key to User.id
- `programming_level`: TEXT, one of ('Beginner', 'Intermediate', 'Advanced')
- `hardware_level`: TEXT, one of ('None', 'Basic', 'Advanced')

**Relationships**:
- A `User` has one `UserBackground`.
- A `UserBackground` belongs to one `User`.

## Chapter

Represents a section of the textbook. This is a conceptual entity for the personalization feature and is not directly stored in the database as a new table. The personalization logic will operate on the existing chapter content.