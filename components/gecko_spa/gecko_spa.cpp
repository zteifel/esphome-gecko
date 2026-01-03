#include "gecko_spa.h"
#include "esphome/core/log.h"
#include <ctime>

namespace esphome {
namespace gecko_spa {

static const char *const TAG = "gecko_spa";

// GO keep-alive message
const uint8_t GeckoSpa::GO_MESSAGE[15] = {
    0x17, 0x00, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x47, 0x4F  // "GO"
};

void GeckoSpa::setup() {
  ESP_LOGI(TAG, "GeckoSpa starting");
  if (reset_pin_) {
    reset_pin_->setup();
    reset_pin_->digital_write(true);  // RST is active LOW, keep HIGH
  }
}

void GeckoSpa::loop() {
  // Handle non-blocking reset pulse completion (100ms)
  if (reset_in_progress_ && (millis() - reset_start_time_ > 100)) {
    if (reset_pin_) {
      reset_pin_->digital_write(true);  // Release reset (HIGH)
    }
    reset_in_progress_ = false;
    ESP_LOGI(TAG, "Arduino reset complete");
  }

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
    if (connected_sensor_)
      connected_sensor_->publish_state(false);
    ESP_LOGW(TAG, "Spa connection lost (timeout)");
    reset_arduino();  // Reset Arduino on disconnect
  }

  // Send GO keep-alive every 23 seconds (triggers handshake sequence)
  if (millis() - last_go_send_time_ > 23000) {
    last_go_send_time_ = millis();
    send_i2c_message(GO_MESSAGE, 15);
    ESP_LOGD(TAG, "Sent GO keep-alive");
  }
}

void GeckoSpa::send_light_command(bool on) {
  uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, 0x52, 0x51,
      0x01, 0x33, (uint8_t)(on ? 0x01 : 0x00), 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent light %s command", on ? "ON" : "OFF");
}

void GeckoSpa::send_pump_command(bool on) {
  uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, 0x52, 0x51,
      0x01, 0x03, (uint8_t)(on ? 0x02 : 0x00), 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent pump %s command", on ? "ON" : "OFF");
}

void GeckoSpa::send_circ_command(bool on) {
  uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, 0x52, 0x51,
      0x01, 0x6B, (uint8_t)(on ? 0x01 : 0x00), 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent circ %s command", on ? "ON" : "OFF");
}

void GeckoSpa::send_program_command(uint8_t prog) {
  if (prog > 4)
    return;
  uint8_t cmd[18] = {
      0x17, 0x0B, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x04, 0x4E, 0x03, 0xD0,
      prog, 0x00};
  cmd[17] = calc_checksum(cmd, 18);
  send_i2c_message(cmd, 18);
  ESP_LOGI(TAG, "Sent program %d command", prog);
}

void GeckoSpa::send_temperature_command(float temp_c) {
  if (temp_c < 26.0 || temp_c > 40.0)
    return;
  uint8_t temp_raw = (uint8_t)((temp_c * 18.0) - 512.0);
  uint8_t cmd[21] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x07, 0x46, 0x52, 0x51,
      0x00, 0x01, 0x02, temp_raw, 0x00};
  cmd[20] = calc_checksum(cmd, 21);
  send_i2c_message(cmd, 21);
  ESP_LOGI(TAG, "Sent temperature %.1f command (raw=%02X)", temp_c, temp_raw);
}

void GeckoSpa::request_status() {
  write_str("PING\n");
}

void GeckoSpa::reset_arduino() {
  if (!reset_pin_) {
    ESP_LOGW(TAG, "Reset pin not configured");
    return;
  }
  if (reset_in_progress_) {
    ESP_LOGD(TAG, "Reset already in progress");
    return;
  }
  ESP_LOGI(TAG, "Resetting Arduino");
  reset_pin_->digital_write(false);  // Pull LOW to reset
  reset_start_time_ = millis();
  reset_in_progress_ = true;
}

uint8_t GeckoSpa::calc_checksum(const uint8_t *data, uint8_t len) {
  uint8_t xor_val = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    xor_val ^= data[i];
  }
  return xor_val;
}

void GeckoSpa::send_i2c_message(const uint8_t *data, uint8_t len) {
  write_str("TX:");
  for (uint8_t i = 0; i < len; i++) {
    char hex[3];
    sprintf(hex, "%02X", data[i]);
    write_str(hex);
  }
  write_str("\n");
}

uint8_t GeckoSpa::hex_to_byte(char high, char low) {
  auto nibble = [](char c) -> uint8_t {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    return 0;
  };
  return (nibble(high) << 4) | nibble(low);
}

void GeckoSpa::process_proxy_message(const char *msg) {
  ESP_LOGD(TAG, "Proxy: %s", msg);

  // RX:<len>:<hex>
  if (strncmp(msg, "RX:", 3) == 0) {
    const char *p = msg + 3;
    int len = atoi(p);

    // Find the colon after length
    while (*p && *p != ':')
      p++;
    if (*p == ':')
      p++;

    // Decode hex to bytes
    uint8_t data[128];
    for (int i = 0; i < len && i < 128; i++) {
      data[i] = hex_to_byte(p[i * 2], p[i * 2 + 1]);
    }

    process_i2c_message(data, len);
  } else if (strcmp(msg, "READY") == 0) {
    ESP_LOGI(TAG, "Arduino proxy ready");
  } else if (strcmp(msg, "I2C_PROXY:V1") == 0) {
    ESP_LOGI(TAG, "Arduino proxy version 1");
  } else if (strcmp(msg, "TX:OK") == 0) {
    ESP_LOGD(TAG, "I2C TX acknowledged");
  } else if (strcmp(msg, "PONG") == 0) {
    ESP_LOGD(TAG, "Proxy ping OK");
  }
}

void GeckoSpa::process_i2c_message(const uint8_t *data, uint8_t len) {
  // Any I2C message means we're connected
  last_i2c_time_ = millis();
  if (!connected_) {
    connected_ = true;
    if (connected_sensor_)
      connected_sensor_->publish_state(true);
    ESP_LOGI(TAG, "Spa connected (I2C traffic detected)");
  }

  // GO message (15 bytes, ends with "GO") - just log it
  if (len == 15 && data[13] == 0x47 && data[14] == 0x4F) {
    ESP_LOGD(TAG, "Received GO message from spa");
    return;
  }

  // Handshake acknowledgment response
  static const uint8_t ACK_MESSAGE[15] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x02
  };

  // 33-byte config file message - send acknowledgment
  if (len == 33) {
    ESP_LOGD(TAG, "Received 33-byte handshake, sending ACK");
    send_i2c_message(ACK_MESSAGE, 15);
    return;
  }

  // 22-byte clock message - parse time and send acknowledgment
  if (len == 22 && data[13] == 0x4B) {  // 0x4B = 'K'
    // Time format: [15]=Day [16]=Month [17]=DayOfWeek [18]=Hour [19]=Min [20]=Sec
    uint8_t day = data[15];
    uint8_t month = data[16];
    uint8_t hour = data[18];
    uint8_t minute = data[19];
    uint8_t second = data[20];

    ESP_LOGD(TAG, "Spa clock: %02d/%02d %02d:%02d:%02d", day, month, hour, minute, second);

    if (spa_time_sensor_) {
      char time_str[20];
      snprintf(time_str, sizeof(time_str), "%02d/%02d %02d:%02d:%02d", day, month, hour, minute, second);
      spa_time_sensor_->publish_state(time_str);
    }

    send_i2c_message(ACK_MESSAGE, 15);
    return;
  }

  // 15-byte "LO" message - handshake complete
  if (len == 15 && data[13] == 0x4C && data[14] == 0x4F) {
    ESP_LOGI(TAG, "Received LO message - handshake complete");
    return;
  }

  // Notification message (77 bytes with byte[6]=0x0B)
  if (len == 77 && data[6] == 0x0B) {
    ESP_LOGD(TAG, "77-byte notification message");
    parse_notification_message(data);
    return;
  }

  // Program status (18 bytes)
  if (len == 18) {
    ESP_LOGI(TAG, "18-byte msg: [1]=%02X [16]=%02X", data[1], data[16]);
    uint8_t prog = data[16];
    if (prog <= 4 && prog != program_id_) {
      program_id_ = prog;
      ESP_LOGI(TAG, "Program from spa: %d", prog);
      if (program_select_) {
        static const char *prog_names[] = {"Away", "Standard", "Energy", "Super Energy", "Weekend"};
        program_select_->publish_state(prog_names[prog]);
      }
    }
    return;
  }

  // Multi-part message handling using byte[9] as continuation flag
  // byte[9] == 0x01: more parts coming, byte[9] == 0x00: last part
  // Header is first 16 bytes (1709000000170A010001000040C75251), strip from subsequent parts
  static const int HEADER_LEN = 16;

  if (len >= 10) {
    bool more_coming = (data[9] == 0x01);

    // Add this part to buffer (strip header from all parts)
    int payload_start = (len > HEADER_LEN) ? HEADER_LEN : 0;
    int payload_len = len - payload_start;
    if (msg_buffer_len_ + payload_len <= sizeof(msg_buffer_)) {
      memcpy(msg_buffer_ + msg_buffer_len_, data + payload_start, payload_len);
      msg_buffer_len_ += payload_len;
    }

    if (more_coming) {
      ESP_LOGD(TAG, "Message part (%d bytes), more coming. Buffer now %d bytes", len, msg_buffer_len_);
      return;
    }

    // Last part received - log complete message in FULL-RX format
    char hex_str[1024];  // Large enough for 405 bytes × 2 = 810 chars + null
    int pos = 0;
    for (int i = 0; i < msg_buffer_len_; i++) {
      pos += sprintf(hex_str + pos, "%02X", msg_buffer_[i]);
    }
    ESP_LOGI(TAG, "FULL-RX:%d:%s", msg_buffer_len_, hex_str);

    // Check if this is a status message (162 bytes after header stripping from all parts)
    // (78-16) + (78-16) + (54-16) = 62 + 62 + 38 = 162 bytes
    // Original byte[17]=0x00 indicates status data, now byte[1] after header strip
    if (msg_buffer_len_ == 162 && msg_buffer_[1] == 0x00) {
      // Log key byte positions including circ at 110
      ESP_LOGI(TAG, "Parsing: [3]=%02X [5]=%02X [21-24]=%02X%02X%02X%02X [53]=%02X [109-112]=%02X%02X%02X%02X",
               msg_buffer_[3], msg_buffer_[5],
               msg_buffer_[21], msg_buffer_[22], msg_buffer_[23], msg_buffer_[24], msg_buffer_[53],
               msg_buffer_[109], msg_buffer_[110], msg_buffer_[111], msg_buffer_[112]);
      parse_status_message(msg_buffer_);
    }

    // Reset buffer for next message
    msg_buffer_len_ = 0;
    return;
  }

  // Short messages (< 11 bytes) - log them
  if (len > 2) {
    char hex_str[64];
    int pos = 0;
    for (int i = 0; i < len && pos < 60; i++) {
      pos += sprintf(hex_str + pos, "%02X", data[i]);
    }
    ESP_LOGI(TAG, "Short msg (%d bytes): %s", len, hex_str);
  }
}

void GeckoSpa::parse_status_message(const uint8_t *data) {
  // Byte positions in 162-byte concatenated message (headers stripped from all 3 parts)
  // Circulation appears at byte 109/110 based on observed on/off sequence
  bool new_standby = (data[3] == 0x03);
  bool new_pump = (data[5] == 0x02) || (data[7] == 0x01);
  bool new_circ = (data[112] == 0x01) || (data[6] & 0x80);  // Byte 112 for manual, byte 6 bit 7 for heating
  bool new_heating = (data[6] & 0x20) || (data[26] & 0x04);
  bool new_light = (data[53] == 0x01);

  // Temperature - only parse if temp data is present
  float new_target = target_temp_;
  float new_actual = actual_temp_;
  bool temp_valid = (data[21] != 0 || data[22] != 0);
  if (temp_valid) {
    uint16_t target_raw = (data[21] << 8) | data[22];
    new_target = target_raw / 18.0;
    uint16_t actual_raw = (data[23] << 8) | data[24];
    new_actual = actual_raw / 18.0;
  }

  // On first status message, publish all states
  bool first = !first_status_received_;
  if (first) {
    first_status_received_ = true;
    ESP_LOGI(TAG, "First status received, publishing all states");
  }

  // Update entities on change (or first message)
  if (first || new_light != light_state_) {
    light_state_ = new_light;
    ESP_LOGI(TAG, "Light: %s", light_state_ ? "ON" : "OFF");
    if (light_switch_)
      light_switch_->publish_state(light_state_);
  }

  if (first || new_pump != pump_state_) {
    pump_state_ = new_pump;
    ESP_LOGI(TAG, "Pump: %s", pump_state_ ? "ON" : "OFF");
    if (pump_switch_)
      pump_switch_->publish_state(pump_state_);
  }

  if (first || new_circ != circ_state_) {
    circ_state_ = new_circ;
    ESP_LOGI(TAG, "Circulation: %s", circ_state_ ? "ON" : "OFF");
    if (circ_switch_)
      circ_switch_->publish_state(circ_state_);
  }

  if (first || new_heating != heating_state_) {
    heating_state_ = new_heating;
    ESP_LOGI(TAG, "Heating: %s", heating_state_ ? "ON" : "OFF");
    update_climate_state();
  }

  if (first || new_standby != standby_state_) {
    standby_state_ = new_standby;
    ESP_LOGI(TAG, "Standby: %s", standby_state_ ? "ON" : "OFF");
    if (standby_sensor_)
      standby_sensor_->publish_state(standby_state_);
  }

  // Only update temperature if valid data was received
  if (temp_valid && (first || abs(new_target - target_temp_) > 0.1 || abs(new_actual - actual_temp_) > 0.1)) {
    target_temp_ = new_target;
    actual_temp_ = new_actual;
    ESP_LOGI(TAG, "Temp: target=%.1f actual=%.1f", target_temp_, actual_temp_);
    update_climate_state();
  }
}

void GeckoSpa::update_climate_state() {
  if (!climate_)
    return;

  climate_->target_temperature = target_temp_;
  climate_->current_temperature = actual_temp_;

  // Set mode based on target vs actual temperature
  if (target_temp_ < actual_temp_) {
    climate_->mode = climate::CLIMATE_MODE_COOL;
  } else {
    climate_->mode = climate::CLIMATE_MODE_HEAT;
  }

  // Set action based on heating flag and temperature comparison
  if (!heating_state_) {
    climate_->action = climate::CLIMATE_ACTION_IDLE;
  } else if (target_temp_ < actual_temp_) {
    climate_->action = climate::CLIMATE_ACTION_COOLING;
  } else {
    climate_->action = climate::CLIMATE_ACTION_HEATING;
  }

  climate_->publish_state();
}

int GeckoSpa::days_since_2000(int day, int month, int year) {
  // Calculate days since Jan 1, 2000
  struct tm tm = {};
  tm.tm_year = 100 + year;  // years since 1900, so 2000 = 100
  tm.tm_mon = month - 1;    // 0-based month
  tm.tm_mday = day;
  time_t target = mktime(&tm);

  struct tm epoch = {};
  epoch.tm_year = 100;  // 2000
  epoch.tm_mon = 0;     // January
  epoch.tm_mday = 1;
  time_t base = mktime(&epoch);

  return (int)((target - base) / 86400);
}

void GeckoSpa::parse_notification_message(const uint8_t *data) {
  // Notification entries start at byte 16, each entry is 6 bytes:
  // [ID] [DD] [MM] [YY] [INTERVAL_LO] [INTERVAL_HI]
  // ID: 0x01=Rinse Filter, 0x02=Clean Filter, 0x03=Change Water, 0x04=Spa Checkup
  for (int i = 0; i < 4; i++) {
    int offset = 16 + (i * 6);
    uint8_t id = data[offset];
    uint8_t reset_day = data[offset + 1];
    uint8_t reset_month = data[offset + 2];
    uint8_t reset_year = data[offset + 3];  // 2-digit year
    uint16_t interval = data[offset + 4] | (data[offset + 5] << 8);

    if (id == 0 || interval == 0)
      continue;

    // Calculate due date: reset_date + interval days
    struct tm reset_tm = {};
    reset_tm.tm_year = 100 + reset_year;  // years since 1900
    reset_tm.tm_mon = reset_month - 1;     // 0-based month
    reset_tm.tm_mday = reset_day + interval;  // mktime normalizes this
    mktime(&reset_tm);  // Normalize the date

    // Format as ISO date string (YYYY-MM-DD)
    char date_str[12];
    snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d",
             1900 + reset_tm.tm_year, reset_tm.tm_mon + 1, reset_tm.tm_mday);

    ESP_LOGI(TAG, "Notification %d: reset=%02d/%02d/%02d interval=%d due=%s",
             id, reset_day, reset_month, reset_year, interval, date_str);

    text_sensor::TextSensor *sensor = nullptr;
    switch (id) {
      case 0x01:
        sensor = rinse_filter_sensor_;
        break;
      case 0x02:
        sensor = clean_filter_sensor_;
        break;
      case 0x03:
        sensor = change_water_sensor_;
        break;
      case 0x04:
        sensor = spa_checkup_sensor_;
        break;
    }

    if (sensor) {
      sensor->publish_state(date_str);
    }
  }
}

// GeckoSpaClimate implementation
void GeckoSpaClimate::setup() {
  this->mode = climate::CLIMATE_MODE_HEAT;
  this->action = climate::CLIMATE_ACTION_IDLE;
  this->target_temperature = 37.0;
  this->current_temperature = NAN;
  this->publish_state();
}

climate::ClimateTraits GeckoSpaClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_supports_current_temperature(true);
  traits.set_supported_modes({climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL});
  traits.set_supports_action(true);
  traits.set_visual_min_temperature(26.0);
  traits.set_visual_max_temperature(40.0);
  traits.set_visual_temperature_step(0.5);
  return traits;
}

void GeckoSpaClimate::control(const climate::ClimateCall &call) {
  if (call.get_target_temperature().has_value()) {
    float temp = *call.get_target_temperature();
    this->target_temperature = temp;
    parent_->send_temperature_command(temp);
  }
  this->publish_state();
}

// GeckoSpaSwitch implementation
void GeckoSpaSwitch::write_state(bool state) {
  if (switch_type_ == "light") {
    parent_->send_light_command(state);
  } else if (switch_type_ == "pump") {
    parent_->send_pump_command(state);
  } else if (switch_type_ == "circulation") {
    parent_->send_circ_command(state);
  }
  // State will be published when spa confirms the change
}

// GeckoSpaSelect implementation
void GeckoSpaSelect::control(const std::string &value) {
  uint8_t prog = 1;
  if (value == "Away")
    prog = 0;
  else if (value == "Standard")
    prog = 1;
  else if (value == "Energy")
    prog = 2;
  else if (value == "Super Energy")
    prog = 3;
  else if (value == "Weekend")
    prog = 4;

  parent_->send_program_command(prog);
  this->publish_state(value);
}

}  // namespace gecko_spa
}  // namespace esphome
