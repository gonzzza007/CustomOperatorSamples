# Instructions for AI agents

## Git
- **Never commit, amend, push, rebase, or otherwise change git history.** The repository owner always commits manually.
- Leave all changes as uncommitted modifications in the working tree. Do not stage files either.
- Read-only git commands (`status`, `diff`, `log`, `show`, `blame`) are fine.

## Building
- Release builds only for anything performance related (`-DCMAKE_BUILD_TYPE=Release`); Debug builds of the simulation SOPs are many times slower.
