#!/usr/bin/env python3
from pathlib import Path
import re
import sys


RISK_PATTERNS = [
    "delay_ms",
    "oled_",
    "DL_UART_Main_transmitDataBlocking",
    "uart0_send_",
    "vofa_send",
    "DL_I2C_",
    "icm42688_",
    "parse",
    "printf",
]


def function_body(text: str, start: int) -> str:
    brace = text.find("{", start)
    if brace < 0:
        return ""
    depth = 0
    for i in range(brace, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[brace : i + 1]
    return text[brace:]


def main() -> int:
    root = Path(__file__).resolve().parents[4]
    files = list((root / "Src").glob("*.c")) + [root / "empty.c"]
    found = []

    handler_re = re.compile(r"\b(?:void|__attribute__\s*\(\([^)]*\)\)\s*void)\s+([A-Za-z0-9_]*IRQHandler|SysTick_Handler)\s*\(")

    for path in files:
        if not path.is_file():
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        lines = text.splitlines()
        for match in handler_re.finditer(text):
            name = match.group(1)
            body = function_body(text, match.start())
            body_start_line = text[: match.start()].count("\n") + 1
            risks = [pat for pat in RISK_PATTERNS if pat in body]
            print(f"ISR {name}: {path.relative_to(root)}:{body_start_line}")
            if risks:
                print(f"  RISK patterns: {', '.join(risks)}")
                found.append((path, name, risks))
            else:
                print("  OK no configured blocking pattern found")

    if found:
        print("FAIL: review ISR risk patterns above. Findings can include false positives.")
        return 1

    print("PASS: no configured blocking patterns found in ISR bodies")
    return 0


if __name__ == "__main__":
    sys.exit(main())
