#!/usr/bin/env python3
from pathlib import Path
import re
import sys


PATTERNS = [
    "motor_set_pwm",
    "speed_pid_set_speed",
    "speed_pid_set_target",
    "speed_pid_set_left_speed",
    "speed_pid_set_right_speed",
    "speed_pid_control_update",
    "angle_control_update",
    "line_track_update",
]


def main() -> int:
    root = Path(__file__).resolve().parents[4]
    files = list((root / "Src").glob("*.c")) + list((root / "Inc").glob("*.h")) + [root / "empty.c"]
    count = 0

    for path in files:
        if not path.is_file():
            continue
        for lineno, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), start=1):
            for pat in PATTERNS:
                if re.search(r"\b" + re.escape(pat) + r"\b", line):
                    print(f"{path.relative_to(root)}:{lineno}: {line.strip()}")
                    count += 1
                    break

    print(f"Total control-writer/update references: {count}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
