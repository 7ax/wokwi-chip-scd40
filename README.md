# Wokwi SCD40 CO2 Sensor Chip

Sensirion SCD40 photoacoustic CO2/Temperature/Humidity sensor simulation for the [Wokwi](https://wokwi.com) simulator.

## Usage

Add to your Wokwi project's `diagram.json`:

```json
{
  "type": "chip-scd40",
  "id": "co2",
  "attrs": { "co2": "800", "temperature": "25", "humidity": "50" }
}
```

Reference in `wokwi.toml`:

```toml
[[chip]]
name = "SCD40"
binary = "chips/scd40.chip.wasm"
```

Or use the GitHub release directly:

```
"github:7ax/wokwi-chip-scd40@1.0.0"
```

## Supported Commands

| Command | Hex    | Description                    |
|---------|--------|--------------------------------|
| start_periodic_measurement | `0x21B1` | Begin periodic CO2 measurement |
| stop_periodic_measurement  | `0x3F86` | Stop measuring                 |
| read_measurement           | `0xEC05` | Read CO2/temp/humidity (9 bytes) |
| get_serial_number          | `0x3682` | Read serial number (9 bytes)   |
| get_data_ready_status      | `0xE4B8` | Check if data is ready (3 bytes) |

I2C address: **0x62** (fixed)

## Interactive Controls

| Control     | Range        | Default |
|-------------|--------------|---------|
| CO2 (ppm)   | 400 - 5000   | 800     |
| Temp (C)    | -10.0 - 60.0 | 25.0    |
| Humidity (%RH) | 0 - 100   | 50      |

## Protocol

The SCD40 uses a **command-based** I2C protocol (not register-based). The master sends a 16-bit command word (MSB first), then reads the response in a separate I2C transaction.

All response data uses Sensirion's CRC8 (polynomial 0x31, init 0xFF) after every 2-byte word.

Compatible with the [Adafruit SCD4x Arduino library](https://github.com/adafruit/Adafruit_SCD4x).

## Building

### GitHub Actions

Push a tag starting with `v` (e.g., `v1.0.0`) to create a release with the compiled WASM binary.

### Local (Docker)

```bash
docker run --rm -v ${PWD}:/src wokwi/builder-clang-wasm:latest make
```

### Local (with WASI SDK)

```bash
make
```

Requires `clang` with WASI sysroot at `/opt/wasi-libc`.

## License

MIT
