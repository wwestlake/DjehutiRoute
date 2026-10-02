# Development Process

This project uses a deliberate research loop so implementation does not run ahead of testing and discussion.

## Working Loop

1. Codex writes or edits code for the agreed slice.
2. Codex stops and waits for explicit permission before building.
3. When cleared, Codex builds the requested target.
4. The user tests the running app.
5. Codex records test notes and observations without immediately fixing them.
6. User and Codex discuss what the notes mean.
7. Codex only executes the next fix or feature pass when the user explicitly says to proceed.

## Current Test Notes Pattern

During test review, capture:

- what worked
- what felt wrong
- UI/interaction issues
- data/model issues
- build/runtime errors
- decisions made during discussion
- items explicitly approved for the next execution pass

## Important Rule

Do not turn test observations directly into code changes. Testing is for notes first. Implementation resumes only after discussion and explicit approval.
