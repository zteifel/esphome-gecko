#pragma once
#include "esphome.h"

// Spa I2C Protocol Handler
// All protocol logic runs on ESP32, Arduino is just an I2C proxy

class SpaProtocol : public Component, public UARTDevice {
 public:
  SpaProtocol(UARTComponent *uart) : UARTDevice(uart) {}

  // Entity setters (called from YAML)
  void set_light_switch(switch_::Switch *sw) { light_switch_ = sw; }
  void set_pump_switch(switch_::Switch *sw) { pump_switch_ = sw; }
  void set_circ_switch(switch_::Switch *sw) { circ_switch_ = sw; }
  void set_program_select(select::Select *sel) { program_select_ = sel; }
  void set_standby_sensor(binary_sensor::BinarySensor *bs) {
    standby_sensor_ = bs;
    bs->publish_state(standby_state_);  // Publish initial state (false)
  }
  void set_connected_sensor(binary_sensor::BinarySensor *bs) {
    connected_sensor_ = bs;
    bs->publish_state(connected_);  // Publish initial state (false)
  }
  void set_climate(climate::Climate *cl) { climate_ = cl; }

  void setup() override {
    ESP_LOGI("spa", "SpaProtocol starting");
  }

  void loop() override {
    // Read UART lines from Arduino proxy
    while (available()) {
      char c = read();
      if (c == '\n' || c == '\r') {
        if (uart_pos_ > 0) {
          uart_buffer_[uart_pos_] = '\0';
          process_proxy_message(uart_buffer_);
          uart_pos_ = 0;
        }
      } else if (uart_pos_ < sizeof(uart_buffer_) - 1) {
        uart_buffer_[uart_pos_++] = c;
      }
    }

    // Check connection timeout (5 minutes)
    if (connected_ && (millis() - last_i2c_time_ > 300000)) {
      connected_ = false;
      if (connected_sensor_) connected_sensor_->publish_state(false);
      ESP_LOGW("spa", "Spa connection lost (timeout)");
    }
  }

  // Command methods (called from switches/climate)
  void send_light_command(bool on) {
    uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, 0x52, 0x51,
      0x01, 0x33, (uint8_t)(on ? 0x01 : 0x00), 0x00
    };
    cmd[19] = calc_checksum(cmd, 20);
    send_i2c_message(cmd, 20);
    ESP_LOGI("spa", "Sent light %s command", on ? "ON" : "OFF");
  }

  void send_pump_command(bool on) {
    uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, 0x52, 0x51,
      0x01, 0x03, (uint8_t)(on ? 0x02 : 0x00), 0x00
    };
    cmd[19] = calc_checksum(cmd, 20);
    send_i2c_message(cmd, 20);
    ESP_LOGI("spa", "Sent pump %s command", on ? "ON" : "OFF");
  }

  void send_circ_command(bool on) {
    uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, 0x52, 0x51,
      0x01, 0x6B, (uint8_t)(on ? 0x01 : 0x00), 0x00
    };
    cmd[19] = calc_checksum(cmd, 20);
    send_i2c_message(cmd, 20);
    ESP_LOGI("spa", "Sent circ %s command", on ? "ON" : "OFF");
  }

  void send_program_command(uint8_t prog) {
    if (prog > 4) return;
    uint8_t cmd[18] = {
      0x17, 0x0B, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x04, 0x4E, 0x03, 0xD0,
      prog, 0x00
    };
    cmd[17] = calc_checksum(cmd, 18);
    send_i2c_message(cmd, 18);
    ESP_LOGI("spa", "Sent program %d command", prog);
  }

  void send_temperature_command(float temp_c) {
    if (temp_c < 26.0 || temp_c > 40.0) return;
    uint8_t temp_raw = (uint8_t)((temp_c * 18.0) - 512.0);
    uint8_t cmd[21] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x07, 0x46, 0x52, 0x51,
      0x00, 0x01, 0x02, temp_raw, 0x00
    };
    cmd[20] = calc_checksum(cmd, 21);
    send_i2c_message(cmd, 21);
    ESP_LOGI("spa", "Sent temperature %.1f command (raw=%02X)", temp_c, temp_raw);
  }

  void request_status() {
    // Send a ping to check connection
    write_str("PING\n");
  }

  // State getters
  bool get_light_state() { return light_state_; }
  bool get_pump_state() { return pump_state_; }
  bool get_circ_state() { return circ_state_; }
  float get_target_temp() { return target_temp_; }
  float get_actual_temp() { return actual_temp_; }
  bool is_heating() { return heating_state_; }

 private:
  // Entity pointers
  switch_::Switch *light_switch_{nullptr};
  switch_::Switch *pump_switch_{nullptr};
  switch_::Switch *circ_switch_{nullptr};
  select::Select *program_select_{nullptr};
  binary_sensor::BinarySensor *standby_sensor_{nullptr};
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  climate::Climate *climate_{nullptr};

  // State
  bool light_state_{false};
  bool pump_state_{false};
  bool circ_state_{false};
  bool heating_state_{false};
  bool standby_state_{false};
  bool connected_{false};
  bool first_status_received_{false};
  uint8_t program_id_{0xFF};
  float target_temp_{0};
  float actual_temp_{0};
  unsigned long last_i2c_time_{0};

  // UART buffer
  char uart_buffer_[512];
  uint16_t uart_pos_{0};

  // GO Response messages
  static constexpr uint8_t GO_RESP1[78] = {
    0x17, 0x09, 0x00, 0x00, 0x00, 0x17, 0x0A, 0x01,
    0x00, 0x01, 0x00, 0x00, 0x40, 0xC7, 0x52, 0x51,
    0x00, 0x00, 0x05, 0x02, 0xA3, 0x02, 0x08, 0x00,
    0x04, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x0C,
    0x00, 0x01, 0x1A, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x01, 0x02, 0x1E, 0x00, 0x00,
    0x00, 0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x0C, 0x01, 0x01, 0x30,
    0x14, 0x78, 0x78, 0x00, 0x28, 0x2A
  };

  static constexpr uint8_t GO_RESP2[78] = {
    0x17, 0x09, 0x00, 0x00, 0x00, 0x17, 0x0A, 0x01,
    0x00, 0x01, 0x00, 0x00, 0x40, 0xC7, 0x52, 0x51,
    0x00, 0x3B, 0x02, 0x04, 0x04, 0x00, 0x28, 0x00,
    0x73, 0x00, 0x90, 0x02, 0xE4, 0x00, 0x01, 0x24,
    0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x0F, 0x02,
    0xE5, 0x00, 0x02, 0x02, 0x02, 0x01, 0x01, 0x05,
    0x03, 0x03, 0x05, 0x05, 0x04, 0x04, 0x01, 0x03,
    0x00, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x07,
    0x06, 0x08, 0x06, 0x07, 0x07, 0xF4, 0x00, 0x02,
    0x00, 0x03, 0x00, 0x00, 0x00, 0xA1
  };

  static constexpr uint8_t GO_RESP3[78] = {
    0x17, 0x09, 0x00, 0x00, 0x00, 0x17, 0x0A, 0x01,
    0x00, 0x01, 0x00, 0x00, 0x40, 0xC7, 0x52, 0x51,
    0x00, 0x76, 0x3C, 0x04, 0x1E, 0x01, 0x0A, 0x30,
    0x40, 0x80, 0x00, 0x83, 0x00, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
    0x30, 0x14, 0x10, 0x30, 0x30, 0x10, 0x0A, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x86
  };

  static constexpr uint8_t GO_RESP4[23] = {
    0x17, 0x09, 0x00, 0x00, 0x00, 0x17, 0x0A, 0x01,
    0x00, 0x01, 0x00, 0x00, 0x09, 0xC7, 0x52, 0x51,
    0x00, 0xB1, 0x00, 0x00, 0x00, 0x01, 0x7E
  };

  static constexpr uint8_t GO_RESP5[2] = { 0x17, 0x0A };

  uint8_t calc_checksum(const uint8_t* data, uint8_t len) {
    uint8_t xor_val = 0;
    for (uint8_t i = 0; i < len - 1; i++) {
      xor_val ^= data[i];
    }
    return xor_val;
  }

  void send_i2c_message(const uint8_t* data, uint8_t len) {
    write_str("TX:");
    for (uint8_t i = 0; i < len; i++) {
      char hex[3];
      sprintf(hex, "%02X", data[i]);
      write_str(hex);
    }
    write_str("\n");
  }

  uint8_t hex_to_byte(char high, char low) {
    auto nibble = [](char c) -> uint8_t {
      if (c >= '0' && c <= '9') return c - '0';
      if (c >= 'A' && c <= 'F') return c - 'A' + 10;
      if (c >= 'a' && c <= 'f') return c - 'a' + 10;
      return 0;
    };
    return (nibble(high) << 4) | nibble(low);
  }

  void process_proxy_message(const char* msg) {
    ESP_LOGD("spa", "Proxy: %s", msg);

    // RX:<len>:<hex>
    if (strncmp(msg, "RX:", 3) == 0) {
      const char* p = msg + 3;
      int len = atoi(p);

      // Find the colon after length
      while (*p && *p != ':') p++;
      if (*p == ':') p++;

      // Decode hex to bytes
      uint8_t data[128];
      for (int i = 0; i < len && i < 128; i++) {
        data[i] = hex_to_byte(p[i*2], p[i*2+1]);
      }

      process_i2c_message(data, len);
    }
    else if (strcmp(msg, "READY") == 0) {
      ESP_LOGI("spa", "Arduino proxy ready");
    }
    else if (strcmp(msg, "I2C_PROXY:V1") == 0) {
      ESP_LOGI("spa", "Arduino proxy version 1");
    }
    else if (strcmp(msg, "TX:OK") == 0) {
      ESP_LOGD("spa", "I2C TX acknowledged");
    }
    else if (strcmp(msg, "PONG") == 0) {
      ESP_LOGD("spa", "Proxy ping OK");
    }
  }

  void process_i2c_message(const uint8_t* data, uint8_t len) {
    // Any I2C message means we're connected
    last_i2c_time_ = millis();
    if (!connected_) {
      connected_ = true;
      if (connected_sensor_) connected_sensor_->publish_state(true);
      ESP_LOGI("spa", "Spa connected (I2C traffic detected)");
    }

    // GO message (15 bytes, ends with "GO")
    if (len == 15 && data[13] == 0x47 && data[14] == 0x4F) {
      ESP_LOGI("spa", "Received GO message, sending response sequence");
      send_go_response();
      return;
    }

    // Program status (18 bytes)
    if (len == 18) {
      ESP_LOGI("spa", "18-byte msg: [1]=%02X [16]=%02X", data[1], data[16]);
      uint8_t prog = data[16];
      if (prog <= 4 && prog != program_id_) {
        program_id_ = prog;
        ESP_LOGI("spa", "Program from spa: %d", prog);
        // Just publish state, don't trigger set_action (would cause loop)
        if (program_select_) {
          static const char* prog_names[] = {"Away", "Standard", "Energy", "Super Energy", "Weekend"};
          program_select_->publish_state(prog_names[prog]);
        }
      }
      return;
    }

    // Status message (78 bytes) - only parse actual status messages
    // byte[6]=0x0A is status, byte[6]=0x0B is config dump (ignore)
    if (len == 78) {
      if (data[6] != 0x0A) {
        ESP_LOGD("spa", "78-byte config msg (ignored): byte[6]=%02X", data[6]);
        return;
      }

      uint8_t msg_type = data[17];
      uint8_t msg_sub = data[18];
      ESP_LOGD("spa", "78-byte status: type=%02X sub=%02X", msg_type, msg_sub);

      if (msg_type == 0x00) {
        // Log key bytes for debugging
        ESP_LOGD("spa", "Status bytes: [19]=%02X [21]=%02X [22]=%02X [23]=%02X [37]=%02X [38]=%02X [39]=%02X [40]=%02X [65]=%02X [69]=%02X",
                 data[19], data[21], data[22], data[23], data[37], data[38], data[39], data[40], data[65], data[69]);
        parse_status_message(data);
      }
      return;
    }

    // Log unhandled messages
    if (len > 2) {
      ESP_LOGD("spa", "Unhandled msg: len=%d first_bytes=%02X%02X%02X", len, data[0], data[1], data[2]);
    }
  }

  void parse_status_message(const uint8_t* data) {
    bool new_standby = (data[19] == 0x03);
    bool new_pump = (data[21] == 0x02) || (data[23] == 0x01);
    bool new_circ = (data[22] & 0x80);  // Bit 7 of byte 22 = circulation
    bool new_heating = (data[22] & 0x20) || (data[42] & 0x04);  // Different bit for heating
    bool new_light = (data[69] == 0x01);

    // Temperature - only parse if temp data is present
    float new_target = target_temp_;  // Keep existing value by default
    float new_actual = actual_temp_;
    bool temp_valid = (data[37] != 0 || data[38] != 0);
    if (temp_valid) {
      uint16_t target_raw = (data[37] << 8) | data[38];
      new_target = target_raw / 18.0;
      uint16_t actual_raw = (data[39] << 8) | data[40];
      new_actual = actual_raw / 18.0;
    }

    // On first status message, publish all states to clear "Unknown"
    bool first = !first_status_received_;
    if (first) {
      first_status_received_ = true;
      ESP_LOGI("spa", "First status received, publishing all states");
    }

    // Update entities on change (or first message)
    if (first || new_light != light_state_) {
      light_state_ = new_light;
      ESP_LOGI("spa", "Light: %s", light_state_ ? "ON" : "OFF");
      if (light_switch_) light_switch_->publish_state(light_state_);
    }

    if (first || new_pump != pump_state_) {
      pump_state_ = new_pump;
      ESP_LOGI("spa", "Pump: %s", pump_state_ ? "ON" : "OFF");
      if (pump_switch_) pump_switch_->publish_state(pump_state_);
    }

    if (first || new_circ != circ_state_) {
      circ_state_ = new_circ;
      ESP_LOGI("spa", "Circulation: %s", circ_state_ ? "ON" : "OFF");
      if (circ_switch_) circ_switch_->publish_state(circ_state_);
    }

    if (first || new_heating != heating_state_) {
      heating_state_ = new_heating;
      ESP_LOGI("spa", "Heating: %s", heating_state_ ? "ON" : "OFF");
    }

    if (first || new_standby != standby_state_) {
      standby_state_ = new_standby;
      ESP_LOGI("spa", "Standby: %s", standby_state_ ? "ON" : "OFF");
      if (standby_sensor_) standby_sensor_->publish_state(standby_state_);
    }

    // Only update temperature if valid data was received
    if (temp_valid && (first || abs(new_target - target_temp_) > 0.1 || abs(new_actual - actual_temp_) > 0.1)) {
      target_temp_ = new_target;
      actual_temp_ = new_actual;
      ESP_LOGI("spa", "Temp: target=%.1f actual=%.1f", target_temp_, actual_temp_);

      // Update climate entity
      if (climate_) {
        climate_->target_temperature = target_temp_;
        climate_->current_temperature = actual_temp_;
        climate_->action = heating_state_ ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
        climate_->publish_state();
      }
    }
  }

  void send_go_response() {
    delay(5);
    send_i2c_message(GO_RESP1, 78);
    delay(5);
    send_i2c_message(GO_RESP2, 78);
    delay(5);
    send_i2c_message(GO_RESP3, 78);
    delay(5);
    send_i2c_message(GO_RESP4, 23);
    for (int i = 0; i < 5; i++) {
      delay(5);
      send_i2c_message(GO_RESP5, 2);
    }
  }
};

// Static member definitions
constexpr uint8_t SpaProtocol::GO_RESP1[78];
constexpr uint8_t SpaProtocol::GO_RESP2[78];
constexpr uint8_t SpaProtocol::GO_RESP3[78];
constexpr uint8_t SpaProtocol::GO_RESP4[23];
constexpr uint8_t SpaProtocol::GO_RESP5[2];

// Simple climate wrapper that uses SpaProtocol
class SpaClimate : public Component, public Climate {
 public:
  SpaClimate(SpaProtocol *proto) : proto_(proto) {}

  void setup() override {
    this->mode = CLIMATE_MODE_HEAT;
    this->action = CLIMATE_ACTION_IDLE;
    this->target_temperature = 37.0;
    this->current_temperature = NAN;
  }

  ClimateTraits traits() override {
    auto traits = climate::ClimateTraits();
    traits.set_supports_current_temperature(true);
    traits.set_supported_modes({CLIMATE_MODE_HEAT});
    traits.set_supports_action(true);
    traits.set_visual_min_temperature(26.0);
    traits.set_visual_max_temperature(40.0);
    traits.set_visual_temperature_step(0.5);
    return traits;
  }

  void control(const ClimateCall &call) override {
    if (call.get_target_temperature().has_value()) {
      float temp = *call.get_target_temperature();
      this->target_temperature = temp;
      proto_->send_temperature_command(temp);
    }
    this->publish_state();
  }

 private:
  SpaProtocol *proto_;
};
