# Gecko Spa Controller

Home Assistant integration for Gecko spa systems using ESP32-S2 and Arduino Nano Clone as an I2C proxy. Tested together with a Gecko IN.YE-3-H3.0 (YE-3-CE) spa controller.

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

**Key Design Decision:** The Arduino acts as a "dumb" I2C proxy. All protocol encoding/decoding happens on the ESP32, which can be updated over WiFi (OTA). This eliminates the need to physically access the spa for firmware updates.

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

   The Arduino Wire library has a default 32-byte I2C buffer, but the spa sends messages up to 78 bytes. You **must** patch the Wire library to increase this buffer.

   1. Install PlatformIO:
      ```bash
      pip install platformio
      ```

   2. Initialize the project to download dependencies:
      ```bash
      cd arduino
      pio run
      ```

   3. **Patch the Wire library** - Edit the twi.h file to increase the buffer size:
      ```bash
      # Find your PlatformIO packages directory (usually ~/.platformio/packages/)
      # Edit the twi.h file:
      nano ~/.platformio/packages/framework-arduino-avr/libraries/Wire/src/utility/twi.h
      ```

      Find the line:
      ```c
      #ifndef TWI_BUFFER_LENGTH
      #define TWI_BUFFER_LENGTH 32
      ```

      Change `32` to `128`:
      ```c
      #ifndef TWI_BUFFER_LENGTH
      #define TWI_BUFFER_LENGTH 128
      ```

   4. Build and upload:
      ```bash
      pio run -t upload
      ```

   > **Why is patching needed?** The `-DTWI_BUFFER_LENGTH=128` build flag in `platformio.ini` should override this value, but some PlatformIO versions don't apply it correctly. Patching the source file directly ensures the buffer is always 128 bytes.

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

# Maintenance reminder sensors
sensor:
  - platform: gecko_spa
    gecko_spa_id: spa
    type: rinse_filter
    name: "Rinse Filter"
    icon: "mdi:air-filter"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: clean_filter
    name: "Clean Filter"
    icon: "mdi:air-filter"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: change_water
    name: "Change Water"
    icon: "mdi:water-sync"

  - platform: gecko_spa
    gecko_spa_id: spa
    type: spa_checkup
    name: "Spa Checkup"
    icon: "mdi:wrench-clock"

# Spa internal clock
text_sensor:
  - platform: gecko_spa
    gecko_spa_id: spa
    type: spa_time
    name: "Spa Time"
    icon: "mdi:clock-outline"
```

5. **Flash and add to Home Assistant**:
   ```bash
   esphome run your-config.yaml
   ```

6. The device will appear in Home Assistant under **Settings → Devices & Services → ESPHome**

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
| Spa Time | Text Sensor | Spa's internal clock (DD/MM HH:MM:SS) |
| Spa Rinse Filter Due | Text Sensor | Due date for filter rinse (YYYY-MM-DD) |
| Spa Clean Filter Due | Text Sensor | Due date for filter clean (YYYY-MM-DD) |
| Spa Change Water Due | Text Sensor | Due date for water change (YYYY-MM-DD) |
| Spa Checkup Due | Text Sensor | Due date for spa checkup (YYYY-MM-DD) |
| Spa Rinse Filter Days | Sensor | Days remaining until filter rinse |
| Spa Clean Filter Days | Sensor | Days remaining until filter clean |
| Spa Change Water Days | Sensor | Days remaining until water change |
| Spa Checkup Days | Sensor | Days remaining until spa checkup |
| Refresh Spa Status | Button | Manually request status update |
| Reset Arduino | Button | Reset the Arduino I2C proxy remotely |

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
![](./pictures/spa_pinouts.png)
![](./pictures/spa_power.png)
![](./pictures/adafruit_esp32s2.png)
<img src="./pictures/arduino_nano_pinout.webp" width="400">
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
    │                                 │       │    │      │
    │                              [2.7kΩ]    │    │      │
    │                                 │       │    │      │
    │                                 ├───────┼────┼──────┤
    │                              [5.6kΩ]    │    │      │
    │                                 │       │    │      │
    │                                GND      │    │      │
    │                                 |       │    │      │
    │  ┌──────────────────────────────┴───────┴────┴───┐  │
    │  │           Adafruit ESP32-S2                   │  │
    │  │                                               │  │
    │  │  GPIO16(RX)  GPIO5(TX)  GPIO17(RST)  USB-C    │  │
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
| `I2C_PROXY:V1\n` | Firmware version on boot |
| `READY\n` | Arduino ready for commands |
| `RX:<len>:<hex>\n` | Received I2C message (length in decimal, data in hex) |
| `TX:OK\n` | I2C transmission acknowledged |
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
   - Byte 15: Day (decimal)
   - Byte 16: Month (decimal)
   - Byte 17: Day of week (0=Sun)
   - Byte 18: Hour (hex, 24h format)
   - Byte 19: Minutes (hex)
   - Byte 20: Seconds (hex)
   - Byte 21: Checksum
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

Maintenance reminder data, sent with byte[6] = 0x0B.

**Entry Format (6 bytes each, starting at byte 16):**

```
[ID] [DD] [MM] [YY] [INTERVAL_LO] [INTERVAL_HI]
```

| Byte | Description |
|------|-------------|
| ID | Notification type (see below) |
| DD | Reset day (decimal, 1-31) |
| MM | Reset month (decimal, 1-12) |
| YY | Reset year (2-digit, e.g., 25 for 2025) |
| INTERVAL_LO | Interval low byte |
| INTERVAL_HI | Interval high byte |

**Notification IDs:**

| ID | Notification |
|----|--------------|
| 0x01 | Rinse Filter |
| 0x02 | Clean Filter |
| 0x03 | Change Water |
| 0x04 | Spa Checkup |

**Interval:** 16-bit little-endian value representing days between reminders.

**Due Date Calculation:** `due_date = reset_date + interval_days`

**Example:** Entry `01 09 12 25 1E 00` = Rinse Filter, reset Dec 9 2025, interval 30 days → due Jan 8 2026

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
17 0A 00 00 00 17 09 00 00 00 00 00 07 46 52 51 00 01 02 [TEMP] [CHK]
                                                         ^^^^   ^^^
                                                         Raw temp value
```

**Temperature Encoding:**

```
TEMP_RAW = (temperature_celsius × 18) - 512
```

| Temperature | Calculation | Raw Value |
|-------------|-------------|-----------|
| 26.0°C | (26 × 18) - 512 = -44 | 0xD4 |
| 36.5°C | (36.5 × 18) - 512 = 145 | 0x91 |
| 37.0°C | (37 × 18) - 512 = 154 | 0x9A |
| 40.0°C | (40 × 18) - 512 = 208 | 0xD0 |

**Example - Set 37°C:**
```
17 0A 00 00 00 17 09 00 00 00 00 00 07 46 52 51 00 01 02 9A [CHK]
```

---

## TODO

- Scheduling of economy intervals and filter cycles in programs (decoding of i2c protocol complete)
- Cleanup notifications fix


## Troubleshooting

### Arduino Hangs After Receiving I2C

- **Most common cause:** I2C buffer too small. The Wire library defaults to 32 bytes, but spa messages are up to 78 bytes. See [Flash the Arduino Nano](#quick-start) for patching instructions or use the precompiled binary.
- Verify the patch was applied: after patching `twi.h`, the line should read `#define TWI_BUFFER_LENGTH 128`
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
