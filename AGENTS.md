# agx_arm_motion Agent Guide

- This package should evolve into a backend-agnostic MoveIt convenience layer for Piper Studio.
- Refactor away hardcoded `agx_arm_gzsim` robot-description and controller assets. The package should select sim or real configuration through explicit profiles or a shared config builder.
- Preserve the current high-level surfaces such as one-shot pose execution and a persistent pose-goal server while generalizing the backend underneath them.
- Keep MoveIt glue centralized here instead of duplicating it in `agx_arm_manipulation` or perception packages.
- Validate new behavior in simulation first, then keep real-hardware support aligned with the same frames and planning groups.