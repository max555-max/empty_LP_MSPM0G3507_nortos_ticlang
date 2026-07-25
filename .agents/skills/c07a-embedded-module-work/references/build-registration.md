# Build Registration

## Current CCS Debug Registration

- Root generated metadata: `Debug/subdir_vars.mk` lists `empty.c`, generated config, and startup source. Evidence A.
- Module metadata: `Debug/Src/subdir_vars.mk` lists current `Src/*.c` files. Evidence A.
- Build rules for module sources are in `Debug/Src/subdir_rules.mk`. Evidence A.

## Required Check For New `.c`

When adding, renaming, or deleting a source file:

1. Run or reproduce `c07a-verify-release/scripts/check_source_registered.py`.
2. Confirm the source appears in CCS Debug metadata.
3. If it is missing, report it rather than editing generated make metadata blindly.
4. Ask whether CCS/SysConfig project regeneration or IDE inclusion is expected.

## Generated File Boundary

Do not treat generated makefiles as the long-term source of truth without understanding CCS project ownership. If generated files are stale, report the mismatch and ask before changing project metadata.
