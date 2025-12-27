# Gecko Spa Controller

Home Assistant integration for Gecko spa systems using ESP32-S2 and Arduino Nano Clone as an I2C proxy.

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

1. [Hardware Build](#hardware-build)
2. [Software Build & Upload](#software-build--upload)
3. [UART Proxy Protocol](#uart-proxy-protocol)
4. [I2C Protocol](#i2c-protocol)
5. [Credits](#credits)

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
#### ESP32-S2 to Arduino Nano Clone (UART)

| ESP32-S2 Pin | Arduino Nano Clone Pin | Notes |
|--------------|------------------|-------|
| GPIO5 (TX) | RX (D0) | Direct connection (3.3V → 5V tolerant) |
| GPIO16 (RX) | TX (D1) | Via voltage divider (5V → 3.3V) |
| GND | GND | Common ground required |

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
    ┌───────────────────┼────┼────┼───────────────────┐
    │                   │    │    │                   │
    │  ┌────────────────┴────┴────┴────────────────┐  │
    │  │           Arduino Nano Clone              │  │
    │  │                                           │  │
    │  │  A4(SDA)  A5(SCL)  GND    TX(D1)  RX(D0)  │  │
    │  └──────────────────────────────┬───────┬────┘  │
    │                                 │       │       │
    │                              [2.7kΩ]    │       │
    │                                 │       │       │
    │                                 ├───────┼───────┤
    │                              [5.6kΩ]    │       │
    │                                 │       │       │
    │                                GND      │       │
    │                                 |       │       │
    │  ┌──────────────────────────────┴───────┴────┐  │
    │  │         Adafruit ESP32-S2                 │  │
    │  │                                           │  │
    │  │  GPIO16(RX)   GPIO5(TX)   GND   USB-C     │  │
    │  └───────────────────────────────────────────┘  │
    │                                                 │
    └─────────────────────────────────────────────────┘
```

---

## Software Build & Upload

### Requirements

- Python 3.8+
- PlatformIO Core
- ESPHome

### Installation

```bash
# Create Python virtual environment
python3 -m venv venv
source venv/bin/activate

# Install ESPHome
pip install esphome

# Install PlatformIO (for Arduino)
pip install platformio
```

### Arduino Nano Clone Firmware

1. Navigate to Arduino directory:
   ```bash
   cd arduino
   ```

2. Build and upload:
   ```bash
   pio run -t upload
   ```

**Critical Build Flag:** The `platformio.ini` includes `-DTWI_BUFFER_LENGTH=128 -DBUFFER_LENGTH=128` to handle 78-byte I2C messages from the spa. Default Arduino Wire buffer is only 32 bytes.

### ESPHome Firmware

1. Navigate to ESPHome directory:
   ```bash
   cd esphome
   ```

2. Create secrets file:
   ```bash
   cp secrets.yaml.template secrets.yaml
   # Edit secrets.yaml with your values
   ```

3. Generate API encryption key:
   ```bash
   openssl rand -base64 32
   ```

4. Build and upload (first time via USB):
   ```bash
   esphome run spa-controller.yaml
   ```

5. Subsequent updates via OTA:
   ```bash
   esphome run spa-controller.yaml --device <IP_ADDRESS>
   ```

### Home Assistant Integration

After uploading, the device will appear in Home Assistant under **Settings → Devices & Services → ESPHome**. Add it using the API encryption key from your `secrets.yaml`.

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

#### GO Keep-Alive Message (15 bytes)

The spa sends this message every ~60 seconds to maintain connection with controllers.

```
17 00 00 00 00 17 09 00 00 00 00 00 01 47 4F
                                       ^^^^
                                       "GO" ASCII
```

**Controller must respond** with a configuration sequence (3x 78-byte messages + 1x 23-byte + 5x 2-byte ACKs) to maintain connection. If no response for 90 seconds, spa considers controller disconnected.

#### Status Message (78 bytes)

Sent periodically with current spa state. Identified by:
- Length: 78 bytes
- Byte[6] = 0x0A (status message; 0x0B = config dump, ignore)
- Byte[17] = 0x00 (message type)

**Key Byte Positions (VERIFIED):**

| Byte | Description | Values |
|------|-------------|--------|
| 6 | Message type | 0x0A = Status, 0x0B = Config dump (ignore) |
| 17 | Sub-type | 0x00 = Status data |
| 19 | Standby state | 0x03 = Standby ON |
| 21 | Pump state | 0x02 = Pump ON |
| 22 | Circulation/Heating | Bit 7 (0x80) = Circ ON, Bit 5 (0x20) = Heating |
| 23 | Pump flag | 0x01 = Pump ON |
| 37-38 | Target temp (raw) | Big-endian, divide by 18.0 for °C |
| 39-40 | Actual temp (raw) | Big-endian, divide by 18.0 for °C |
| 42 | Heating flag | Bit 2 (0x04) = Heating |
| 69 | Light state | 0x01 = Light ON |
| 77 | Checksum | XOR of bytes 0-76 |

**Note:** Some status messages have zeros in temperature fields (bytes 37-40). Preserve existing temperature values when this occurs.

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

#### GO Response Sequence

When the spa sends a GO message, the controller must respond with:

1. **Response 1** (78 bytes) - Configuration type 0x00/0x05
2. **Response 2** (78 bytes) - Configuration type 0x3B/0x02
3. **Response 3** (78 bytes) - Configuration type 0x76/0x3C
4. **Response 4** (23 bytes) - Type 0xB1/0x00
5. **Response 5** (2 bytes) - ACK `17 0A` × 5 times

Each response should be sent with a 5ms delay between messages.

---

## Troubleshooting

### Arduino Hangs After Receiving I2C

- Ensure buffer size flags are set: `-DTWI_BUFFER_LENGTH=128 -DBUFFER_LENGTH=128`
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

## License

MIT License - See LICENSE file for details.
