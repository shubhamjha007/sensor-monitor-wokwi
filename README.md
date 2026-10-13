# Simulated Sensor Monitoring System (ESP32 on Wokwi)

Firmware for an ESP32 sensor node, simulated in the browser with Wokwi.

   **[Run the simulation on Wokwi](https://wokwi.com/projects/477794880583281665)**

## Features
- NTC temperature sensor read through the 12-bit ADC (Beta equation)
- Button handled by a GPIO **interrupt**: the ISR only debounces and sets a flag; work happens in `loop()`
- **Finite state machine**: `IDLE` -> `SAMPLING` -> `ALERT` (with hysteresis on the way back)
- **UART command-line interface** at 115200 baud
- Red LED lights in `ALERT`

## Wiring
| Part | Pin | Connects to |
|---|---|---|
| NTC sensor | VCC / GND / OUT | 3V3 / GND / GPIO 34 |
| Push button | one side / other side | GPIO 25 / GND (internal pull-up used) |
| LED + 220 ohm | anode via resistor / cathode | GPIO 26 / GND |

## Run it
1. Go to wokwi.com, start a new **ESP32** project.
2. Replace `sketch.ino` and `diagram.json` with the files here.
   If `diagram.json` shows an error, delete it and wire the parts by hand using the table above.
3. Press Play. Open the Serial Monitor.
4. Click the NTC sensor in the diagram and drag its temperature slider above the threshold to trigger `ALERT`.

## Commands
```
help | status | read | set <degC> | start | stop
```

## State machine
| From | Event | To |
|---|---|---|
| IDLE | button / `start` | SAMPLING |
| SAMPLING | button / `stop` | IDLE |
| SAMPLING | T > threshold | ALERT |
| ALERT | T < threshold - 2 C | SAMPLING |
| ALERT | button / `stop` | IDLE |

## Design notes
- ISR is minimal (debounce + flag) so it stays short and does not call blocking functions. Shared variables are `volatile`.
- Sampling uses `millis()` timing rather than `delay()`, so the CLI and button stay responsive.
- Hysteresis stops the state flapping when the temperature hovers near the threshold.
- UART input is read non-blockingly into a bounded line buffer; over-long lines are discarded.

## Possible extensions
- Replace `millis()` sampling with a hardware timer interrupt
- Move to FreeRTOS tasks (sensor task, CLI task) with a queue
- Reuse the ring buffer and packet parser from *Embedded C Firmware Utilities* for a binary UART protocol
