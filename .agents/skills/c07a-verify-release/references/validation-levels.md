# Validation Levels

Use the lowest level that is fully supported by evidence.

| Level | Meaning | Evidence Needed |
|---|---|---|
| L0 | File reading only | Repository files read and cited |
| L1 | Static checks | Read-only scripts/searches run and results reviewed |
| L2 | Build passed | User-authorized Debug build completed successfully |
| L3 | Single-module hardware communication confirmed | User-provided or explicitly authorized hardware communication evidence |
| L4 | Lifted-wheel or bench validation | User-authorized motor/bench test evidence |
| L5 | Low-speed ground validation | User-authorized low-speed real-car run evidence |
| L6 | Full competition-scene validation | User-provided full-course or competition-scenario evidence |

## Rules

- L3 and above require hardware evidence supplied by the user or collected only after explicit authorization.
- Build output alone can reach at most L2.
- Static scripts alone can reach at most L1.
- Do not infer real-car readiness from source presence, generated pins, or successful compilation.
