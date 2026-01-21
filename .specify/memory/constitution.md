<!-- SYNC IMPACT REPORT
Version change: N/A -> 1.0.0
Added sections: All principles and sections for the AI-Spec Driven Technical Book project
Removed sections: None
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# AI-Spec Driven Technical Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-Driven, Reproducible Development
All development follows Spec-Kit Plus methodology with clearly defined specs, plans, and tasks. Every feature must be reproducible and testable. All implementation work must reference precise code locations and follow the spec-plan-task workflow.

### Technical Accuracy and Originality
All content must be accurate, verifiable, and plagiarism-free. Code must follow production best practices. All content must be original and properly cited where applicable. Technical claims must be fact-checked and verifiable.

### Clear Writing for CS/Software Engineering Audience
All content must be written with clarity and precision for computer science and software engineering professionals. Complex concepts must be explained with practical examples. Documentation must be accessible yet technically accurate.

### Practical, Deployable Outcomes
Every feature and component must result in practical, deployable outcomes. The book must be deployable on GitHub Pages, and the RAG chatbot must function correctly with grounded responses. All implementations must be production-ready.

### Grounded RAG Responses
RAG chatbot responses must be grounded strictly in indexed book content. Selected-text Q&A must use only user-selected text as context. No hallucinations or responses outside the indexed knowledge base are allowed.

### Free-Tier Compatibility and Security
All technology choices must be compatible with free-tier services only. No hardcoded secrets are allowed. All deployments must follow security best practices and be reproducible.

## Technology Stack Requirements

Technology stack includes Docusaurus + GitHub Pages for hosting, Spec-Kit Plus and Claude Code for development, FastAPI for backend services, OpenAI Agents/ChatKit SDKs for AI functionality, Neon Serverless Postgres for database, and Qdrant Cloud (free tier) for vector storage.

## Development Workflow

Development follows Spec-Kit Plus workflow: spec → plan → tasks → implementation. All changes must be tested and validated before deployment. Code reviews must verify compliance with all constitutional principles.

## Governance

Constitution supersedes all other practices. All PRs and reviews must verify compliance with constitutional principles. Amendments require documentation and approval. All development must follow Spec-Kit Plus methodology.

**Version**: 1.0.0 | **Ratified**: 2026-01-19 | **Last Amended**: 2026-01-19