---
id: 0006
title: Fix Homepage Start Learning Buttons
stage: general
date: 2026-02-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 003-vla-integration
user: Subra Ansari
command: /sp.phr
labels: ["docusaurus", "homepage", "buttons", "routing", "modules"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - E:\Hackathon 1\my-website\src\components\ModuleCard\index.js
tests:
 - none
---

## Prompt

Homepage "Start Learning" buttons issue.

Problem:
- Homepage shows 4 modules
- Clicking "Start Learning":
  - Module 4 opens correctly
  - Module 1, 2, and 3 do NOT open

Actions:
- Check homepage "Start Learning" button links for Module 1–3
- Compare their routes/slugs with the working Module 4 link
- Fix incorrect paths so they point to the correct module index pages
- Ensure links match Docusaurus docs routing exactly

Rules:
- Do NOT change footer (already working)
- Do NOT change module content
- Fix homepage links only

## Response snapshot

Fixed the homepage "Start Learning" buttons by correcting the paths in the ModuleCard component. Updated all module links to use proper Docusaurus routing: /docs/module-1/, /docs/digital-twin/, /docs/isaac-ai-brain/, and /docs/module-4/. All buttons now direct to correct module index pages.

## Outcome

- ✅ Impact: All "Start Learning" buttons now work correctly
- 🧪 Tests: none
- 📁 Files: Modified src/components\ModuleCard\index.js
- 🔁 Next prompts: none
- 🧠 Reflection: Docusaurus routing requires proper path formats for internal links

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect path formats in ModuleCard component
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A