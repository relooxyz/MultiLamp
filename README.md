# MultiLamp Controller

**Created:** ‎June ‎27, ‎2023  
**Last Modified:** September ‎4, ‎2024

This Arduino-based project enables environmental monitoring and IR-based appliance control. It integrates a DHT temperature and humidity sensor, Panasonic AC IR control, signal modulation for lights, and digital I/O expansion via the MCP23017 chip.

## Features

- **DHTx**  
  Reads temperature and humidity from a DHT sensor and calculates heat index. Uses precise timing with hardware timers for robust polling.

- **IRSender**  
  Sends IR signals compatible with Panasonic air conditioners using custom-defined protocols. Supports toggle buttons (Eion, Patrol, Powerfull Quiet), user-defined packets, and low-level bit manipulation.

- **Light**  
  Provides blinking/pulsing signal patterns through an MCP-controlled GPIO pin, enabling custom LED signal sequences.

- **MCP**  
  A driver for the MCP23017 I²C GPIO expander. Provides digitalWrite functionality and port management for both GPIOA and GPIOB.

## File Structure

| File            | Purpose |
|-----------------|---------|
| `MultiLamp.ino` | Main Arduino sketch integrating all modules |
| `DHTx.h/.cpp`   | Custom DHT11/22 sensor interface with timer-based polling |
| `IRSender.h/.cpp` | Panasonic IR protocol implementation and utilities |
| `Light.h/.cpp`  | Signal driver for toggling lights or LEDs |
| `MCP.h/.cpp`    | MCP23017 GPIO expander interface via I2C |

## Hardware Requirements

- ESP8266 (or compatible board)
- DHT11/DHT22 sensor
- MCP23017 I/O expander
- IR LED (connected through MCP)
- Lights/LEDs for signaling

## Dependencies

- `Wire.h` (I²C communication)
- `user_interface.h` (for ESP8266 timer APIs)
- `DHT.h` (DHT sensor support)

## Getting Started

1. Connect your sensors, IR LED, and lights to the correct pins on the MCP and ESP.
2. Upload `MultiLamp.ino` to your microcontroller.
3. Modify timing or protocol constants in `IRSender.cpp` and `DHTx.cpp` if using different AC brands or sensor models.

## License

Unlicense (Feel free to reuse this code in any way preferred)
