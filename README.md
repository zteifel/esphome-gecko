<p align="center"><img src="https://github.com/zteifel/esphome-gecko/blob/master/logo_small.png" alt="Smart Spa Heating" width="400"><img src="https://github.com/zteifel/smart-spa-heating/blob/main/logo_small.png" alt="Smart Spa Heating" width="400"></p>


Home Assistant integration for Gecko spa systems using ESP32-S2 and Arduino Nano Clone as an I2C proxy. Tested together with a Gecko IN.YE-3-H3.0 (YE-3-CE) spa controller. Feel free to combine with my home assistant integration for smart heating based on electricty price [Smart Spa Heating Integration](https://github.com/zteifel/smart-spa-heating).

## Architecture

```
┌─────────────────┐     UART      ┌──────────────────┐     I2C     ┌─────────────┐
│   ESP32-S2      │◄─────────────►│  Arduino Nano    │◄───────────►│  Gecko Spa  │
│                 │   (115200)    │   (I2C Proxy)    │   (0x17)    │             │
│ - All protocol  │               │                  │             │             │
│   logic         │  TX:hex\n     │ - Hex decode     │             │             │
│ - Home Assistant│  ─────────►   │ - Forward to I2C │             │             │
│ - OTA updates   │               │                  │             │             │
│                 │  RX:len:hex\n │ - Forward I2C RX │             │             │
│                 │  ◄────────    │   as hex         │             │             │
└─────────────────┘               └──────────────────┘             └─────────────┘
```

**Key Design Decision:** The Arduino acts as a "dumb" I2C proxy. All protocol encoding/decoding happens on the ESP32, which can be updated over WiFi (OTA). This eliminates the need to physically access the spa for firmware updates and make uses of the arduino i2c capability how handlings the multi-master and large i2c messages.

## Table of Contents

1. [Installation](#installation)
2. [Hardware Build](#hardware-build)
3. [UART Proxy Protocol](#uart-proxy-protocol)
4. [I2C Protocol](#i2c-protocol)
5. [Buy me a coffee](#buy-me-a-coffee)
6. [TODO](#todo)
7. [Troubleshooting](#troubleshoot)
8. [Credits](#credits)

---

## Installation

### Quick Start

1. **Set up hardware** - See [Hardware Build](#hardware-build) section below

2. **Flash the Arduino Nano** - Choose one of the following options:

   **Option A: Use Precompiled Binary (Recommended)**

   Download `arduino-i2c-proxy-atmega328p.hex` from the [GitHub Releases](https://github.com/zteifel/esphome-gecko/releases) page (or from `arduino/` folder).

   Flash using avrdude:
   ```bash
   # Install avrdude (Ubuntu/Debian)
   sudo apt install avrdude

   # For Nano Clone (new bootloader) - 115200 baud
   avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b115200 -D -Uflash:w:arduino-i2c-proxy-atmega328p.hex:i

   # For Original Arduino Nano (old bootloader) - 57600 baud
   avrdude -v -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:arduino-i2c-proxy-atmega328p.hex:i
   ```

   > **Tip:** Most cheap Nano clones from AliExpress/Amazon use the new bootloader (115200 baud). If upload fails, try 57600 baud for original Nanos with old bootloader.

   **Option B: Build from Source with PlatformIO**

   The stock Arduino Wire library uses 32-byte buffers, but the spa sends messages up to 78 bytes.
   This project ships a patched copy in `arduino/lib/Wire` with 128-byte buffers, so no manual
   patching is needed (the build fails with a clear error if the buffers are too small).

   ```bash
   pip install platformio
   cd arduino
   pio run -t upload
   ```

   > **Note:** Older versions of these instructions only patched `twi.h`. That is not enough:
   > `Wire.h` hard-codes `BUFFER_LENGTH 32`, so 78-byte messages overflowed `TwoWire::rxBuffer`
   > and corrupted the memory after it. The precompiled hex was always built correctly.

   Once installed, later Arduino updates can be done over WiFi, see
   [Updating the Arduino over WiFi](#updating-the-arduino-over-wifi).

3. **Create a secrets.yaml file** with your credentials:
   ```yaml
   wifi_ssid: "YourWiFiName"
   wifi_password: "YourWiFiPassword"
   api_encryption_key: "generate-with-openssl-rand-base64-32"
   ota_password: "your-ota-password"
   ```
   Generate the API key with: `openssl rand -base64 32`

4. **Create your ESPHome configuration** - This component can be installed directly from GitHub:

```yaml
substitutions:
  device_name: spa-controller

esphome:
  name: ${device_name}
  friendly_name: Spa Controller

esp32:
  board: featheresp32-s2
  framework:
    type: arduino

# Import Gecko Spa component from GitHub
external_components:
  - source: github://zteifel/esphome-gecko
    components: [gecko_spa]

logger:
  level: DEBUG
  baud_rate: 0

wifi:
  ssid: !secret wifi_ssid
  password: !secret wifi_password

api:
  encryption:
    key: !secret api_encryption_key

ota:
  platform: esphome
  password: !secret ota_password

# UART connection to Arduino I2C Proxy
uart:
  id: arduino_uart
  tx_pin: GPIO5
  rx_pin: GPIO16
  baud_rate: 115200
  rx_buffer_size: 512

# Gecko Spa component
gecko_spa:
  id: spa
  uart_id: arduino_uart

# Climate control
climate:
  - platform: gecko_spa
    gecko_spa_id: spa
    name: "Spa"

# Switches
switch:
  - platform: gecko_spa
    gecko_spa_id: spa
    type: light
    name: "Spa Light"
    icon: "mdi:lightbulb"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: pump
    name: "Spa Pump"
    icon: "mdi:pump"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: circulation
    name: "Spa Circulation"
    icon: "mdi:rotate-3d-variant"

# Program selector
select:
  - platform: gecko_spa
    gecko_spa_id: spa
    name: "Spa Program"
    icon: "mdi:format-list-bulleted"

# Status sensors
binary_sensor:
  - platform: gecko_spa
    gecko_spa_id: spa
    type: standby
    name: "Spa Standby"
    icon: "mdi:power-standby"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: connected
    name: "Spa Connected"
    device_class: connectivity

# Spa internal clock and maintenance reminder due dates
text_sensor:
  - platform: gecko_spa
    gecko_spa_id: spa
    type: spa_time
    name: "Spa Time"
    icon: "mdi:clock-outline"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: rinse_filter
    name: "Rinse Filter Due"
    icon: "mdi:air-filter"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: clean_filter
    name: "Clean Filter Due"
    icon: "mdi:air-filter"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: change_water
    name: "Change Water Due"
    icon: "mdi:water-sync"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: spa_checkup
    name: "Spa Checkup Due"
    icon: "mdi:wrench-clock"
```

5. **Flash and add to Home Assistant**:
   ```bash
   esphome run your-config.yaml
   ```

6. The device will appear in Home Assistant under **Settings → Devices & Services → ESPHome**

### Updating the Arduino over WiFi

The ESP32 can pass its UART through to the Arduino, so the Arduino can be reflashed without
opening the spa. Add a `serial_proxy` on the Arduino UART and link it to `gecko_spa`:

```yaml
gecko_spa:
  id: spa
  uart_id: arduino_uart
  reset_pin: GPIO17
  serial_proxy_id: arduino_serial

serial_proxy:
  - id: arduino_serial
    name: "Arduino"
    port_type: TTL
    uart_id: arduino_uart
```

While a client is connected to the proxy, `gecko_spa` pauses the spa protocol and resets the
Arduino into its bootloader. `tools/arduino_ota.py` bridges the proxy to avrdude
(needs `pip install aioesphomeapi pyyaml` and avrdude; PlatformIO's copy is found automatically):

```bash
# Read-only test, also backs up the current firmware
python3 tools/arduino_ota.py --host <esp-ip> --secrets esphome/secrets.yaml --baud 57600 \
  -- -U flash:r:backup.hex:i

# Flash
python3 tools/arduino_ota.py --host <esp-ip> --secrets esphome/secrets.yaml --baud 57600 \
  -- -U flash:w:arduino/arduino-i2c-proxy-atmega328p.hex:i
```

Use `--baud 57600` for the old Nano bootloader and `--baud 115200` for the new (Optiboot) one.
If sync fails at one speed, try the other. The bootloader is never overwritten, so a failed
flash can simply be retried the same way.

### Home Assistant Entities

After installation, you'll have these entities:

| Entity | Type | Description |
|--------|------|-------------|
| Spa | Climate | Temperature control with current/target display |
| Spa Light | Switch | Control spa light |
| Spa Pump | Switch | Control main pump |
| Spa Circulation | Switch | Control circulation pump |
| Spa Program | Select | Choose program (Away, Standard, Energy, Super Energy, Weekend) |
| Spa Standby | Binary Sensor | Standby mode status |
| Spa Connected | Binary Sensor | Connection status to spa |
| Spa Time | Text Sensor | Spa's internal clock (YYYY-MM-DD HH:MM:SS, or HH:MM:SS if the pack sends no date) |
| Spa Rinse Filter Due | Text Sensor | Due date for filter rinse (YYYY-MM-DD) |
| Spa Clean Filter Due | Text Sensor | Due date for filter clean (YYYY-MM-DD) |
| Spa Change Water Due | Text Sensor | Due date for water change (YYYY-MM-DD) |
| Spa Checkup Due | Text Sensor | Due date for spa checkup (YYYY-MM-DD) |
| Refresh Spa Status | Button | Manually request status update |
| Reset Arduino | Button | Reset the Arduino I2C proxy remotely |
| Spa Arduino Freezes | Sensor | Times the Arduino stopped answering and was reset (since ESP boot) |

---

## Hardware Build

### Components Required

| Component | Description | Notes |
|-----------|-------------|-------|
| Adafruit Feather ESP32-S2 | WiFi microcontroller | Handles Home Assistant communication |
| Arduino Nano Clone | I2C bridge | 5V logic for spa I2C bus |
| Voltage Divider Resistors | 2.7kΩ + 5.6kΩ | Level shifting Arduino TX → ESP32 RX |
| Dupont Wires | Various | Connections between components |

### Pin Connections
#### ESP32-S2 to Arduino Nano Clone (UART + Reset)

| ESP32-S2 Pin | Arduino Nano Clone Pin | Notes |
|--------------|------------------|-------|
| GPIO5 (TX) | RX (D0) | Direct connection (3.3V → 5V tolerant) |
| GPIO16 (RX) | TX (D1) | Via voltage divider (5V → 3.3V) |
| GPIO17 | RST | Arduino reset (directly, no resistor needed) |
| GND | GND | Common ground required |

> **Arduino Reset:** GPIO17 directly connects to the Arduino RST pin. This allows resetting the Arduino from Home Assistant without physical access. The RST pin is active-LOW and has an internal pull-up resistor.

#### Voltage Divider Circuit (Arduino TX → ESP32 RX)

```
Arduino TX (D1) ----[2.7kΩ]----+---- ESP32 GPIO16 (RX)
                               |
                            [5.6kΩ]
                               |
                              GND
```

Output voltage: ~2.7V (within ESP32 3.3V logic threshold)

#### Arduino Nano Clone to Spa I2C Bus

| Arduino Nano Clone Pin | Spa Connector | Notes |
|------------------|---------------|-------|
| A4 (SDA) | SDA | I2C Data |
| A5 (SCL) | SCL | I2C Clock |
| GND | GND | Common ground |

**Important:** Do NOT connect Arduino VCC to spa - power Arduino separately via USB or external supply.

### Wiring Diagram
Credits to agittins for the pictures

<img src="./pictures/spa_pinouts.png" width="500"><img src="./pictures/spa_power.png" width="400">
<img src="./pictures/arduino_nano_pinout.webp" width="350"><img src="./pictures/adafruit_esp32s2.png" width="550">

```
                    ┌─────────────────┐
                    │   Gecko Spa     │
                    │   Motherboard   │
                    │                 │
                    │  SDA  SCL  GND  │
                    └───┬────┬────┬───┘
                        │    │    │
    ┌───────────────────┼────┼────┼───────────────────────┐
    │                   │    │    │                       │
    │  ┌────────────────┴────┴────┴────────────────────┐  │
    │  │             Arduino Nano Clone                │  │
    │  │                                               │  │
    │  │  A4(SDA)  A5(SCL)  GND   TX(D1)  RX(D0)  RST  │  │
    │  └──────────────────────────────┬───────┬────┬───┘  │
    │                        |        │       │    │      │
    │                        |     [2.7kΩ]    │    │      │
    │                        |        │       │    │      │
    │                        |─[5.6kΩ]┼       |    |      │ 
    │                        |        │       │    |      │
    │                        |        │       │    │      │
    │                        |        |       │    │      │
    │                        |        |       │    │      │
    │  ┌─────────────────────┴────────┴───────┴────┴───┐  │
    │  │                    GND     GPIO16 GPIO5 GPIO17│  │
    │  │                                               │  │
    │  │               Adafruit ESP32-S2               │  │
    │  └───────────────────────────────────────────────┘  │
    │                                                     │
    └─────────────────────────────────────────────────────┘
```

---

## UART Proxy Protocol

### Overview

The Arduino acts as a transparent I2C proxy. The ESP32 sends raw I2C bytes as hex strings, and the Arduino forwards them to the I2C bus. Similarly, I2C messages received by the Arduino are sent to the ESP32 as hex strings.

### Message Format

- **Baud Rate:** 115200
- **Data Bits:** 8
- **Parity:** None
- **Stop Bits:** 1
- **Line Ending:** `\n` (LF)

### Commands (ESP32 → Arduino)

| Command | Description |
|---------|-------------|
| `TX:<hex>\n` | Send hex bytes to I2C bus at address 0x17 |
| `PING\n` | Health check |

**Example - Send light ON command:**
```
TX:170A0000001709000000000646525101330163\n
```

### Responses (Arduino → ESP32)

| Message | Description |
|---------|-------------|
| `I2C_PROXY:V2\n` | Firmware version on boot |
| `READY\n` | Arduino ready for commands |
| `RX:<len>:<hex>\n` | Received I2C message (length in decimal, data in hex) |
| `TX:OK\n` | I2C transmission acknowledged |
| `TX:ERR:I2C:<code>\n` | I2C transmission failed (V2+). Code from `endTransmission()`: 2=address NACK, 3=data NACK, 4=other (e.g. lost arbitration) |
| `TX:ERR:TIMEOUT\n` | I2C operation timed out after 25 ms and the TWI hardware was reset (V2+) |
| `TX:ERR:INVALID_HEX\n` | Invalid hex string |
| `TX:ERR:TOO_LONG\n` | Message exceeds 128 bytes |
| `PONG\n` | Response to PING |

**Example - Received 78-byte status message:**
```
RX:78:17090000001709...4F\n
```

### Protocol Logic

All spa protocol logic (GO responses, command encoding, status parsing) runs on the ESP32 in `spa_protocol.h`. This allows OTA updates without physical access to the spa.

---

## I2C Protocol

### Overview

The Gecko spa uses I2C for communication between components. The spa motherboard and external controllers share address **0x17** in a multi-master configuration.

### Bus Configuration

| Parameter | Value |
|-----------|-------|
| I2C Address | 0x17 (23 decimal) |
| Bus Speed | 100kHz (standard mode) |
| Configuration | Multi-master |
| Pull-ups | 4.7kΩ (typically on spa bus) |

### Multi-Master Operation

Both the spa motherboard and the Arduino controller use address 0x17. The spa sends status updates to this address, and the controller sends commands to this address. This allows bidirectional communication without address conflicts.

### Message Checksums

Most messages use XOR checksum of bytes 0 to (length-2), stored in the last byte.

```c
uint8_t calcChecksum(uint8_t* data, uint8_t len) {
    uint8_t xorVal = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
        xorVal ^= data[i];
    }
    return xorVal;
}
```

---

### Messages FROM Spa (Status Updates)

#### GO Keep-Alive & Handshake Protocol

The controller sends GO every 60 seconds to trigger a handshake sequence.

**GO Message (15 bytes):**
```
17 00 00 00 00 17 09 00 00 00 00 00 01 47 4F
                                       ^^^^
                                       "GO" ASCII
```

**Handshake Sequence:**
After GO is sent, the spa responds with messages that must be acknowledged:

1. **Spa sends 33-byte config message** (contains XML filename like `inYT_C82.xml`)
2. **Controller replies with ACK** (15 bytes): `17 0A 00 00 00 17 09 00 00 00 00 00 01 00 02`
3. **Spa sends another 33-byte config message**
4. **Controller replies with ACK**
5. **Spa sends 22-byte clock message** (byte[13] = 0x4B = 'K', contains date/time)
   - Byte 14: Year (from 2000, e.g. 0x1A = 2026)
   - Byte 15: Month
   - Byte 16: Day
   - Byte 17: Day of week (0=Sun)
   - Byte 18: Hour (hex, 24h format)
   - Byte 19: Minutes (hex)
   - Byte 20: Seconds (hex)
   - Byte 21: Checksum
   - Example: `… 4B 1A 0B 08 00 0F 11 19 5F` = 2026-11-08, Sunday, 15:17:25. Some packs send `FF` for the date bytes.
6. **Controller replies with ACK**
7. **Spa sends 15-byte "LO" message**: bytes[13-14] = 0x4C 0x4F = "LO"
8. **Handshake complete** - status messages now flow normally

**Important:** Without proper handshake acknowledgment, commands may not receive immediate status responses.

#### Status Message (Multi-Part, 162 bytes concatenated)

Status data is sent as a **multi-part message** split across 3 I2C transmissions.

**Multi-Part Message Structure:**

| Part | Raw Size | Header | Payload | Description |
|------|----------|--------|---------|-------------|
| 1 | 78 bytes | 16 bytes | 62 bytes | First status part |
| 2 | 78 bytes | 16 bytes | 62 bytes | Second status part |
| 3 | 54 bytes | 16 bytes | 38 bytes | Final status part |
| **Total** | 210 bytes | 48 bytes | **162 bytes** | Concatenated payload |

**Continuation Flag (Byte 9 in raw message):**
- `0x01` = More parts coming
- `0x00` = Last part of message

**Header (16 bytes, stripped from each part):**
```
17 09 00 00 00 17 0A 01 00 XX 00 00 YY ZZ 52 51
```
Where XX = continuation flag (byte 9), YY ZZ = length/type info

**Message Identification (in concatenated 162-byte payload):**
- Byte[1] = 0x00 indicates status data

**Key Byte Positions (VERIFIED in 162-byte concatenated payload):**

| Byte | Description | Values |
|------|-------------|--------|
| 1 | Data type | 0x00 = Status data |
| 3 | Standby | 0x03 = Standby ON |
| 5 | Pump state | 0x02 = Pump ON |
| 6 | Heat flags | Bit 7 (0x80) = Circ during heating, Bit 5 (0x20) = Heating |
| 7 | Pump flag | 0x01 = Pump ON |
| 21-22 | Target temp (raw) | Big-endian, divide by 18.0 for °C |
| 23-24 | Actual temp (raw) | Big-endian, divide by 18.0 for °C |
| 26 | Heating flag | Bit 2 (0x04) = Heating |
| 53 | Light state | 0x01 = Light ON |
| 112 | Circulation | 0x01 = Circulation ON (manual toggle) |

**Example:** Raw temp bytes `02 9A` = 0x029A = 666 / 18.0 = **37.0°C**

#### Configuration Message (Multi-Part, 405 bytes concatenated)

Periodic configuration/settings dump sent by the spa, typically after handshake.

**Multi-Part Message Structure:**

Similar to status messages, this is sent as multiple I2C transmissions with 16-byte headers stripped and payloads concatenated to form a 405-byte message.

**Known Byte Positions (PARTIALLY DECODED):**

| Byte | Description | Values |
|------|-------------|--------|
| 3-4 | Target temperature (raw) | Big-endian, divide by 18.0 for °C |

**Example:** Bytes 3-4 = `02 9A` = 0x029A = 666 / 18.0 = **37.0°C**

**Note:** This message likely contains program schedules, filter cycle settings, notification intervals, and other configuration data. Further reverse engineering is needed to decode additional fields.

---

#### Program Status Message (18 bytes)

Indicates current program selection.

```
17 0B 00 00 00 17 09 00 00 00 00 00 04 4E 03 D0 [PROG] [CHK]
                                                  ^^^^
                                                  Program ID
```

**Program IDs:**

| ID | Program |
|----|---------|
| 0x00 | Away |
| 0x01 | Standard |
| 0x02 | Energy |
| 0x03 | Super Energy |
| 0x04 | Weekend |

---

#### Notification Message (77 bytes)

Maintenance reminder data, sent with byte[6] = 0x0B. The spa sends it when a reminder is
**reset** (on the keypad, or by the spa itself), not when the menu is only viewed, so the
component stores the last values in flash and restores them after a reboot.

**Entry Format (6 bytes each, starting at byte 16):**

```
[ID] [YY] [MM] [DD] [INTERVAL_LO] [INTERVAL_HI]
```

| Byte | Description |
|------|-------------|
| ID | Notification type (see below) |
| YY | Reset year (from 2000, e.g. 0x1A = 2026) |
| MM | Reset month (1-12) |
| DD | Reset day (1-31) |
| INTERVAL_LO | Interval low byte |
| INTERVAL_HI | Interval high byte |

The date is year-month-day, the same order as the clock message. If your dates look wrong,
`notif_date_format: D-M-Y` on `gecko_spa` switches back to the old interpretation.

**Notification IDs:**

| ID | Notification |
|----|--------------|
| 0x01 | Rinse Filter |
| 0x02 | Clean Filter |
| 0x03 | Change Water |
| 0x04 | Spa Checkup |

**Interval:** 16-bit little-endian value representing days between reminders.

**Due Date Calculation:** `due_date = reset_date + interval_days`

**Example:** Entry `01 1A 0B 08 1E 00` = Rinse Filter, reset 2026-11-08, interval 30 days → due 2026-12-08

---

### Messages TO Spa (Commands)

#### On/Off Command (20 bytes)

Controls light, pump, and circulation.

```
17 0A 00 00 00 17 09 00 00 00 00 00 06 46 52 51 01 [FUNC] [STATE] [CHK]
                                                    ^^^^   ^^^^^   ^^^
                                                    Function ID    Checksum
```

**Function IDs:**

| ID | Function | ON State | OFF State |
|----|----------|----------|-----------|
| 0x33 | Light | 0x01 | 0x00 |
| 0x03 | Pump | 0x02 | 0x00 |
| 0x6B | Circulation | 0x01 | 0x00 |

**Note:** Pump uses 0x02 for ON state, not 0x01.

**Example - Light ON:**
```
17 0A 00 00 00 17 09 00 00 00 00 00 06 46 52 51 01 33 01 [CHK]
```

#### Program Select Command (18 bytes)

Changes the spa program.

```
17 0B 00 00 00 17 09 00 00 00 00 00 04 4E 03 D0 [PROG] [CHK]
```

**Pre-calculated Commands:**

| Program | Command (hex) |
|---------|---------------|
| Away | `17 0B 00 00 00 17 09 00 00 00 00 00 04 4E 03 D0 00 9B` |
| Standard | `17 0B 00 00 00 17 09 00 00 00 00 00 04 4E 03 D0 01 9A` |
| Energy | `17 0B 00 00 00 17 09 00 00 00 00 00 04 4E 03 D0 02 99` |
| Super Energy | `17 0B 00 00 00 17 09 00 00 00 00 00 04 4E 03 D0 03 98` |
| Weekend | `17 0B 00 00 00 17 09 00 00 00 00 00 04 4E 03 D0 04 9F` |

#### Temperature Set Command (21 bytes)

Sets the target temperature for the spa.

```
17 0A 00 00 00 17 09 00 00 00 00 00 07 46 52 51 00 01 [TEMP_HI] [TEMP_LO] [CHK]
                                                      ^^^^^^^^^^^^^^^^^^
                                                      16-bit big-endian raw temp
```

**Temperature Encoding:**

Same format as the temperatures in the status message: a big-endian 16-bit value in 1/18 °C units.

```
TEMP_RAW = temperature_celsius × 18
```

| Temperature | Calculation | Raw Value | Bytes |
|-------------|-------------|-----------|-------|
| 26.0°C | 26 × 18 = 468 | 0x01D4 | `01 D4` |
| 36.5°C | 36.5 × 18 = 657 | 0x0291 | `02 91` |
| 37.0°C | 37 × 18 = 666 | 0x029A | `02 9A` |
| 40.0°C | 40 × 18 = 720 | 0x02D0 | `02 D0` |

> Between 28.5°C and 42.6°C the high byte is always `0x02`, which is why it was earlier mistaken for a constant.

**Example - Set 37°C:**
```
17 0A 00 00 00 17 09 00 00 00 00 00 07 46 52 51 00 01 02 9A [CHK]
```

---

## TODO

- Scheduling of economy intervals and filter cycles in programs (decoding of i2c protocol complete)
- Optional spa clock sync from Home Assistant (the spa clock drifts, and reminders roll over at the spa's midnight)


## Troubleshooting

### Arduino Hangs After Receiving I2C

- **Most common cause:** I2C buffer too small. The stock Wire library uses 32 bytes, but spa messages are up to 78 bytes. Use the precompiled binary or build with the bundled `arduino/lib/Wire`.
- **Freezes under traffic:** firmware V1 could hang forever inside Wire when an I2C send collided with spa traffic (the AVR Wire timeout is off by default). V2 enables a 25 ms timeout and reports `TX:ERR` instead. The ESP32 also resets an Arduino that doesn't answer a send within 1 s, and counts these in the *Spa Arduino Freezes* sensor.
- Do NOT use `digitalRead()` on SDA/SCL pins
- Do NOT use hardware watchdog

### ESP32 Not Receiving UART

- Verify voltage divider output is >2.5V
- Check common ground between Arduino and ESP32
- Verify correct GPIO pins (GPIO5 TX, GPIO16 RX)

### Spa Not Responding

- Ensure GO response sequence is sent within 60 seconds
- Verify I2C address 0x17
- Check I2C pull-up resistors

---

## Credits
Credit to https://github.com/agittins for the pictures and initial research.

## Buy me a coffee
https://www.buymeacoffee.com/zteifel
## License

MIT License - See LICENSE file for details.
