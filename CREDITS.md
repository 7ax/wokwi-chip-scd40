# Credits

## Wokwi Custom Chip SDK

- **Source:** https://github.com/wokwi/inverter-chip
- **License:** MIT
- **What was borrowed:** `wokwi-api.h` header file, Makefile structure, GitHub Actions workflow template
- **Modifications:** Makefile adapted for SCD40 build with additional warning flags (`-Wall -Wextra`). Workflow adapted for SCD40 source files.

## SCD40 Protocol Reference

- **Source:** Sensirion SCD4x Datasheet (https://sensirion.com/products/catalog/SCD40)
- **License:** N/A (datasheet used as protocol reference only)
- **What was used:** I2C command protocol specification (all 20 commands from Table 9), CRC8 algorithm parameters (poly=0x31, init=0xFF), temperature/humidity conversion formulas, command execution times, state machine behavior
