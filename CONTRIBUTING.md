# Contributing

## Branches
- `main`: always runnable. Protected: no direct pushes, PR + 1 approval.
- `dev`: integration branch. Branch from here, PR back here. Merge `dev` to `main` at each milestone (M1..M5).
- Feature branches: `<area>/<short-name>`, e.g. `ui/grid-canvas`, `ros/mission-fsm`, `be/ingest-api`.

## Commits and PRs
- Small commits, imperative message: `Add telemetry schema`.
- One PR = one topic. Link the issue (`Closes #12`). Another member reviews; Jojo reviews anything touching `docs/contract/`.
- Never commit secrets: API keys go in `.env` (gitignored), see `.env.example`.

## Contract changes
`docs/contract/` is the single source of truth between ROS, backend and dashboard.
Before M2 (13 Nov): edit freely but announce in the group chat.
After M2: change only via PR labelled `contract-change`, approved by all three.

## Issue board
Columns: Backlog / This week / In progress / Review / Done. Labels: `ui`, `ros`, `backend`, `docs`, `contract-change`, `milestone:M1`..`M5`.
