#!/usr/bin/env python3
from pathlib import Path
import re
import sys


def main() -> int:
    root = Path(__file__).resolve().parents[4]
    src_dir = root / "Src"
    vars_file = root / "Debug" / "Src" / "subdir_vars.mk"

    if not src_dir.is_dir():
        print("ERROR: missing Src directory")
        return 2
    if not vars_file.is_file():
        print("ERROR: missing Debug/Src/subdir_vars.mk")
        return 2

    src_files = {p.name for p in src_dir.glob("*.c")}
    text = vars_file.read_text(encoding="utf-8", errors="replace")
    registered = set(re.findall(r"\.\./Src/([^\\/\s]+\.c)", text))

    missing = sorted(src_files - registered)
    stale = sorted(registered - src_files)
    ok = sorted(src_files & registered)

    print("Registered Src/*.c:")
    for name in ok:
        print(f"  OK {name}")

    if missing:
        print("Missing from Debug/Src/subdir_vars.mk:")
        for name in missing:
            print(f"  MISSING {name}")

    if stale:
        print("Registered but file is absent:")
        for name in stale:
            print(f"  STALE {name}")

    if missing or stale:
        return 1

    print("PASS: all current Src/*.c files are registered in Debug/Src/subdir_vars.mk")
    return 0


if __name__ == "__main__":
    sys.exit(main())
