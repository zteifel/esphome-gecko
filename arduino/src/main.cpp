#include <Arduino.h>
#include <Wire.h>

// The spa sends messages up to 78 bytes. The stock AVR Wire library uses 32-byte buffers and
// silently overflows them, so this project ships a patched copy in lib/Wire.
static_assert(BUFFER_LENGTH >= 128, "Wire BUFFER_LENGTH too small: use the patched lib/Wire");

#define SPA_ADDRESS 0x17
#define SERIAL_BAUD 115200
// Give up on a stuck I2C operation after 25 ms and reset the TWI hardware, instead of
// waiting forever (the AVR Wire default). Multi-master collisions with the spa can
// otherwise leave Wire spinning in a while loop, freezing the proxy.
#define WIRE_TIMEOUT_US 25000

// I2C receive buffer
volatile uint8_t i2cBuffer[128];
volatile uint8_t i2cBufferLen = 0;
volatile bool newI2CMessage = false;

// UART receive buffer
char uartBuffer[512];
uint16_t uartBufferPos = 0;

// Hex conversion helpers
uint8_t hexCharToNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return 0;
}

uint8_t hexToByte(char high, char low) {
    return (hexCharToNibble(high) << 4) | hexCharToNibble(low);
}

void printHex(uint8_t b) {
    if (b < 16) Serial.print('0');
    Serial.print(b, HEX);
}

// I2C event handlers
void receiveEvent(int numBytes) {
    i2cBufferLen = 0;
    while (Wire.available() && i2cBufferLen < 128) {
        i2cBuffer[i2cBufferLen++] = Wire.read();
    }
    newI2CMessage = true;
}

void requestEvent() {
    // Spa does repeated-start + read, respond with 2 zero bytes
    Wire.write((uint8_t)0x00);
    Wire.write((uint8_t)0x00);
}

// Send bytes to I2C bus
void sendToI2C(uint8_t* data, uint8_t len) {
    // Temporarily become master to send
    Wire.end();
    Wire.begin();

    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(SPA_ADDRESS);
    Wire.write(data, len);
    // 0=ok 1=too long 2=NACK on address 3=NACK on data 4=other (e.g. lost arbitration) 5=timeout
    uint8_t result = Wire.endTransmission(false);

    // Read 2 bytes (spa expects this)
    if (result == 0) {
        Wire.requestFrom((uint8_t)SPA_ADDRESS, (uint8_t)2);
        while (Wire.available()) Wire.read();
    }
    bool timedOut = Wire.getWireTimeoutFlag();

    // Return to slave mode
    Wire.end();
    Wire.begin(SPA_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    if (timedOut) {
        Serial.println("TX:ERR:TIMEOUT");
    } else if (result != 0) {
        Serial.print("TX:ERR:I2C:");
        Serial.println(result);
    } else {
        Serial.println("TX:OK");
    }
}

// Process UART command
void processUartCommand(const char* cmd) {
    // TX:<hex bytes>
    if (strncmp(cmd, "TX:", 3) == 0) {
        const char* hex = cmd + 3;
        uint16_t hexLen = strlen(hex);

        if (hexLen < 2 || hexLen % 2 != 0) {
            Serial.println("TX:ERR:INVALID_HEX");
            return;
        }

        uint8_t data[128];
        uint8_t dataLen = hexLen / 2;

        if (dataLen > 128) {
            Serial.println("TX:ERR:TOO_LONG");
            return;
        }

        for (uint8_t i = 0; i < dataLen; i++) {
            data[i] = hexToByte(hex[i*2], hex[i*2+1]);
        }

        sendToI2C(data, dataLen);
    }
    // PING - health check
    else if (strcmp(cmd, "PING") == 0) {
        Serial.println("PONG");
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    delay(100);

    Serial.println("I2C_PROXY:V2");

    Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
    Wire.begin(SPA_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    Serial.println("READY");
}

void loop() {
    // Forward I2C messages to UART as hex
    if (newI2CMessage) {
        newI2CMessage = false;

        Serial.print("RX:");
        Serial.print(i2cBufferLen);
        Serial.print(":");
        for (uint8_t i = 0; i < i2cBufferLen; i++) {
            printHex(i2cBuffer[i]);
        }
        Serial.println();
    }

    // Process UART commands
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (uartBufferPos > 0) {
                uartBuffer[uartBufferPos] = '\0';
                processUartCommand(uartBuffer);
                uartBufferPos = 0;
            }
        } else if (uartBufferPos < sizeof(uartBuffer) - 1) {
            uartBuffer[uartBufferPos++] = c;
        }
    }
}
