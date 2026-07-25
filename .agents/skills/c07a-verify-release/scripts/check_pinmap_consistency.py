#!/usr/bin/env python3
from pathlib import Path
import re
import sys


KNOWN_DOC_CONFLICTS = {
    "motor": ["PA26", "PA24", "PA25", "PA27", "PB25", "PB24"],
    "encoder": ["PA14", "PA15", "PA16", "PA17"],
    "gray": ["PA22", "PB20"],
    "icm42688": ["SPI1", "PB09", "PB08", "PB07", "PB06"],
    "buzzer": ["PB05"],
    "led": ["PB22"],
}


def read(path: Path) -> str:
    if not path.is_file():
        raise FileNotFoundError(str(path))
    return path.read_text(encoding="utf-8", errors="replace")


def main() -> int:
    root = Path(__file__).resolve().parents[4]
    syscfg = read(root / "empty.syscfg")
    gen_h = read(root / "Debug" / "ti_msp_dl_config.h")
    gen_c = read(root / "Debug" / "ti_msp_dl_config.c")
    wiring_path = root / "接线说明.md"
    wiring = wiring_path.read_text(encoding="utf-8", errors="replace") if wiring_path.is_file() else ""

    errors = []
    warnings = []

    generated_names = [
        "PWM_INST",
        "I2C_ICM42688_INST",
        "UART0_INST",
        "UART_1_INST",
        "OLED_SCL_PIN_SCL_PIN",
        "OLED_SDA_PIN_SDA_PIN",
        "ENCODER_E1A_PIN",
        "GRAY_SERIAL_DAT_PIN",
    ]
    for name in generated_names:
        in_h = name in gen_h
        in_source = any(name in p.read_text(encoding="utf-8", errors="replace")
                        for p in list((root / "Src").glob("*.c")) + list((root / "Inc").glob("*.h")) + [root / "empty.c"]
                        if p.is_file())
        if not in_h:
            errors.append(f"Generated macro missing from ti_msp_dl_config.h: {name}")
        elif not in_source and name not in ("ENCODER_E1A_PIN", "GRAY_SERIAL_DAT_PIN"):
            warnings.append(f"Generated macro exists but was not observed in source usage: {name}")

    if "UART2.peripheral.txPin.$assign         = \"PB6\"" in syscfg and "GPIO_UART_1_TX_PIN" in gen_h:
        print("OK software config: UART_1 TX PB6 present in SysConfig/generated files")
    else:
        errors.append("UART_1 PB6 software mapping not confirmed")

    if "UART2.peripheral.rxPin.$assign         = \"PB7\"" in syscfg and "GPIO_UART_1_RX_PIN" in gen_h:
        print("OK software config: UART_1 RX PB7 present in SysConfig/generated files")
    else:
        errors.append("UART_1 PB7 software mapping not confirmed")

    pin_assigns = re.findall(r'pin\.\$assign\s*=\s*"([PA-B][0-9]+)"', syscfg)
    duplicates = sorted({p for p in pin_assigns if pin_assigns.count(p) > 1})
    for pin in duplicates:
        warnings.append(f"Duplicate pin assignment token observed in empty.syscfg: {pin}")

    if wiring:
        for topic, tokens in KNOWN_DOC_CONFLICTS.items():
            if any(token in wiring for token in tokens):
                warnings.append(f"Wiring note may conflict with current software configuration in topic: {topic}")
    else:
        warnings.append("No wiring note file found")

    print("Pinmap consistency report")
    print("Software configuration facts are not physical wiring proof.")
    for msg in warnings:
        print(f"WARNING: {msg}")
    for msg in errors:
        print(f"ERROR: {msg}")

    if errors:
        return 1
    print("PASS: no fatal SysConfig/generated/source macro mismatch found by this lightweight scan")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except FileNotFoundError as exc:
        print(f"ERROR: missing file: {exc}")
        sys.exit(2)
