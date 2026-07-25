# API Change Checklist

Use before changing a public header, global variable, macro, or function signature.

## Required Searches

- Declaration in `Inc/*.h`.
- Definition in `Src/*.c` or `empty.c`.
- All call sites in `Src/`, `Inc/`, and `empty.c`.
- Any indirect user through Bluetooth, UART command, or control modules.

## Check Items

- Function name and return type match.
- Parameter order, signedness, width, pointer constness, and null behavior are documented or locally obvious.
- Units and scaling are stated for physical/control values.
- Structure layout changes do not silently break callers.
- Macro changes do not conflict with generated macros.
- ISR-callable functions do not block.
- Removed APIs have no remaining call paths.
- Added APIs are declared in the matching header and implemented in one source file.

## Reporting

State which declarations, definitions, and call sites were checked. If hardware validation is still needed, mark it Evidence C.
