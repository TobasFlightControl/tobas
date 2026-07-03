# AGENTS.md

This file gives repository-specific instructions for AI coding agents working on Tobas.
Follow [CONTRIBUTING.md](./CONTRIBUTING.md) first; this document only turns those contribution rules into an agent workflow.

## Scope

- These instructions apply to the whole `tobas` repository.
- If a more specific `AGENTS.md` exists in a subdirectory, follow that file for files under that subdirectory.
- Do not overwrite or revert user changes unless the user explicitly asks.

## Before Editing

- Read the relevant source files and nearby tests before changing behavior.
- Prefer small, focused changes that match the existing package structure.

## Validation

- Run the smallest useful validation for the change first.
- When feasible before handoff, run:

  ```bash
  pre-commit run --files path/to/file
  ```

- For package-level C++ or ROS 2 changes, run the relevant `colcon build` command
  when practical and report any environment or permission failures separately from compile failures.

## Git And Review

- Follow [GIT_GUIDELINE.md](./GIT_GUIDELINE.md) for branch names, commit messages, and pull requests.
- Do not create commits, branches, tags, or pull requests unless the user asks.
- In final responses, summarize changed files, validation performed, and any known remaining risk.
