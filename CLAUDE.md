# Garden Automation — Project Instructions

## Branching Strategy

### Major changes to `src/main.cpp`
A **major change** includes:
- Adding or removing a significant feature (new sensor, new automation logic, new communication protocol)
- Refactoring the overall structure (task scheduling, state machines, loop logic)
- Breaking changes to existing behavior

**→ Create a new branch before starting work:**
```bash
git checkout -b feature/<short-description>
# example: git checkout -b feature/soil-moisture-sensor
```

Merge back to `main` via a pull request once the change is tested and stable (authorization given by Lucien)

---

### Minor changes to `src/main.cpp`
A **minor change** includes:
- Bug fixes
- Adjusting thresholds, timings, or constants
- Small improvements to existing logic (without altering the overall structure)

**→ Continue on the current branch.** No new branch needed.

---

## Confirmation Before Editing
Before making any modification to any file, Claude must explicitly confirm to the user that it has actually read this CLAUDE.md file, by stating: **"I have read CLAUDE.md."** — then proceed.

---

## General Rules
- Never commit directly to `main` for major changes.
- Commit messages should be short and descriptive (e.g. `fix: adjust watering threshold`, `feat: add DHT22 temperature sensor`).
- Always verify the build compiles before committing (`pio run`).