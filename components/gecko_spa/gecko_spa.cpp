#include "gecko_spa.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <ctime>

namespace esphome {
namespace gecko_spa {

static const char *const TAG = "gecko_spa";

// GO keep-alive message
const uint8_t GeckoSpa::GO_MESSAGE[15] = {
    0x17, 0x00, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x01, 0x47, 0x4F  // "GO"
};

#ifdef USE_ESP_IDF
// Reply sent when the spa reads from us after writing (mirrors the Arduino
// bridge's requestEvent(), which always answered with two zero bytes).
const uint8_t GeckoSpa::I2C_ACK_BYTES[2] = {0x00, 0x00};
#endif

void GeckoSpa::setup() {
  ESP_LOGI(TAG, "GeckoSpa starting");
  if (reset_pin_) {
    reset_pin_->setup();
    reset_pin_->digital_write(true);  // RST is active LOW, keep HIGH
  }
#ifdef USE_ESP_IDF
  if (uart_ == nullptr) {
    i2c_slave_install_();
  }
#endif
}

void GeckoSpa::on_shutdown() {
#ifdef USE_ESP_IDF
  if (uart_ == nullptr) {
    i2c_slave_uninstall_();
  }
#endif
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

#ifdef USE_GECKO_SPA_UART
  if (uart_ != nullptr) {
    // Transport A: read UART lines from Arduino proxy
    auto *u = static_cast<uart::UARTComponent *>(uart_);
    while (u->available()) {
      uint8_t c;
      u->read_byte(&c);
      if (c == '\n' || c == '\r') {
        if (uart_pos_ > 0) {
          uart_buffer_[uart_pos_] = '\0';
          process_proxy_message(uart_buffer_);
          uart_pos_ = 0;
        }
      } else if (uart_pos_ < sizeof(uart_buffer_) - 1) {
        uart_buffer_[uart_pos_++] = (char) c;
      }
    }
  }
#endif
#ifdef USE_ESP_IDF
  if (uart_ == nullptr) {
    // Transport B: drain messages received while acting as I2C slave
    // (unsolicited writes from the spa's own controller board - GO/ACK/
    // status/config/notifications all arrive this way).
    if (i2c_rx_queue_ != nullptr) {
      uint32_t recv_len;
      while (xQueueReceive(i2c_rx_queue_, &recv_len, 0) == pdTRUE) {
        uint8_t len = (uint8_t) std::min<uint32_t>(recv_len, sizeof(i2c_rx_buf_));
        ESP_LOGD(TAG, "I2C RX (raw, %u bytes): %s", len, format_hex_pretty(i2c_rx_buf_, len).c_str());
        process_i2c_message(i2c_rx_buf_, len);
      }
    }

    // Log the on-request ack write from the main task - the ISR that
    // actually performs it (i2c_slave_on_request_cb) can't safely call
    // ESP_LOG*.
    uint32_t ack_sent = i2c_ack_sent_count_.load(std::memory_order_relaxed);
    if (ack_sent != i2c_ack_logged_count_) {
      ESP_LOGD(TAG, "I2C TX (ack on request, x%lu): %s", ack_sent - i2c_ack_logged_count_,
               format_hex_pretty(I2C_ACK_BYTES, sizeof(I2C_ACK_BYTES)).c_str());
      i2c_ack_logged_count_ = ack_sent;
    }
  }
#endif

  // Check connection timeout (1 minute without I2C traffic)
  if (millis() - last_i2c_time_ > 60000) {
    if (connected_) {
      connected_ = false;
      if (connected_sensor_)
        connected_sensor_->publish_state(false);
      ESP_LOGW(TAG, "Spa connection lost (timeout)");
      reset_retry_count_ = 0;
      reset_arduino();
    } else if (!reset_in_progress_) {
      // Not connected and not mid-reset: retry with backoff
      // Retry intervals: 30s, 60s, 120s, then every 120s (max 5 retries before giving up)
      uint32_t backoff = 30000UL * (1 << std::min(reset_retry_count_, (uint8_t)2));
      if (reset_retry_count_ < 5 && (millis() - reset_start_time_ > backoff)) {
        ESP_LOGW(TAG, "Arduino recovery retry %d/%d (backoff %ds)",
                 reset_retry_count_ + 1, 5, backoff / 1000);
        reset_arduino();
      }
    }
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
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, config_version_, status_version_,
      0x01, 0x33, (uint8_t)(on ? 0x01 : 0x00), 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent light %s command", on ? "ON" : "OFF");
}

void GeckoSpa::send_circ_command(bool on) {
  uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, config_version_, status_version_,
      0x01, 0x6B, (uint8_t)(on ? 0x01 : 0x00), 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent circ %s command", on ? "ON" : "OFF");
}

void GeckoSpa::send_pump1_command(uint8_t state) {
  // P1 function ID: 0x03
  // State: 0=OFF, 2=ON/HIGH (P1 uses 0x02 for ON, not 0x01)
  uint8_t state_val = (user_demand_state_ & 0xFC) | ((state == 0) ? 0x00 : 0x02);

  uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, config_version_, status_version_,
      0x01, 0x03, state_val, 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent P1 state=%d command (val=0x%02X)", state, state_val);
}

void GeckoSpa::send_pump2_command(uint8_t state) {
  // P2 function ID: 0x03
  // State: 0=OFF, 8=ON/HIGH (P2 uses 0x08 for ON, not 0x01)
  uint8_t state_val = (user_demand_state_ & 0xF3) | ((state == 0) ? 0x00 : 0x08);

  uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, config_version_, status_version_,
      0x01, 0x03, state_val, 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent P2 state=%d command (val=0x%02X)", state, state_val);
}

void GeckoSpa::send_pump3_command(uint8_t state) {
  // P3 function ID: 0x03
  // State: 0=OFF, 32=ON/HIGH (P3 uses 0x20 for ON, not 0x01)
  uint8_t state_val = (user_demand_state_ & 0xCF) | ((state == 0) ? 0x00 : 0x20);

  uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, config_version_, status_version_,
      0x01, 0x03, state_val, 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent P3 state=%d command (val=0x%02X) [EXPERIMENTAL]", state, state_val);
}

void GeckoSpa::send_pump4_command(uint8_t state) {
  // P4 function ID: 0x03
  // State: 0=OFF, 128=ON/HIGH (P4 uses 0x80 for ON, not 0x01)
  uint8_t state_val = (user_demand_state_ & 0x3F) | ((state == 0) ? 0x00 : 0x80);

  uint8_t cmd[20] = {
      0x17, 0x0A, 0x00, 0x00, 0x00, 0x17, 0x09, 0x00,
      0x00, 0x00, 0x00, 0x00, 0x06, 0x46, config_version_, status_version_,
      0x01, 0x03, state_val, 0x00};
  cmd[19] = calc_checksum(cmd, 20);
  send_i2c_message(cmd, 20);
  ESP_LOGI(TAG, "Sent P4 state=%d command (val=0x%02X) [EXPERIMENTAL]", state, state_val);
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
      0x00, 0x00, 0x00, 0x00, 0x07, 0x46, config_version_, status_version_,
      0x00, 0x01, 0x02, temp_raw, 0x00};
  cmd[20] = calc_checksum(cmd, 21);
  send_i2c_message(cmd, 21);
  ESP_LOGI(TAG, "Sent temperature %.1f command (raw=%02X)", temp_c, temp_raw);
}

void GeckoSpa::request_status() {
#ifdef USE_GECKO_SPA_UART
  if (uart_ != nullptr) {
    static_cast<uart::UARTComponent *>(uart_)->write_str("PING\n");
    return;
  }
#endif
#ifdef USE_ESP_IDF
  // No dedicated "give me your status now" command is known for this
  // protocol - the spa pushes status on its own schedule. Sending the same
  // GO keep-alive used automatically every 23s is the best available manual
  // nudge to (re)trigger that sequence on demand.
  ESP_LOGI(TAG, "Manual status request - sending GO");
  send_i2c_message(GO_MESSAGE, 15);
#endif
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
  ESP_LOGI(TAG, "Resetting Arduino (attempt %d)", reset_retry_count_ + 1);
  arduino_ready_ = false;
  reset_pin_->digital_write(false);  // Pull LOW to reset
  reset_start_time_ = millis();
  reset_in_progress_ = true;
  reset_retry_count_++;
}

uint8_t GeckoSpa::calc_checksum(const uint8_t *data, uint8_t len) {
  uint8_t xor_val = 0;
  for (uint8_t i = 0; i < len - 1; i++) {
    xor_val ^= data[i];
  }
  return xor_val;
}

void GeckoSpa::send_i2c_message(const uint8_t *data, uint8_t len) {
#ifdef USE_GECKO_SPA_UART
  if (uart_ != nullptr) {
    // Transport A: hex-encode over UART to the Arduino proxy
    auto *u = static_cast<uart::UARTComponent *>(uart_);
    u->write_str("TX:");
    for (uint8_t i = 0; i < len; i++) {
      char hex[3];
      sprintf(hex, "%02X", data[i]);
      u->write_str(hex);
    }
    u->write_str("\n");
    return;
  }
#endif

#ifdef USE_ESP_IDF
  // Transport B: brief role swap - tear down the slave device, become
  // master just long enough to write the command and read the spa's 2-byte
  // ack, then resume listening. Mirrors the Arduino bridge's sendToI2C()
  // Wire.end()/Wire.begin() dance above.
  ESP_LOGD(TAG, "send_i2c_message: %s", format_hex_pretty(data, len).c_str());

  i2c_slave_uninstall_();

  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = I2C_PORT;
  bus_cfg.sda_io_num = i2c_sda_pin_;
  bus_cfg.scl_io_num = i2c_scl_pin_;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.flags.enable_internal_pullup = true;

  i2c_master_bus_handle_t bus = nullptr;
  esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
    i2c_slave_install_();
    return;
  }

  i2c_device_config_t dev_cfg = {};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = i2c_address_;
  dev_cfg.scl_speed_hz = 100000;

  i2c_master_dev_handle_t dev = nullptr;
  err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(err));
    i2c_del_master_bus(bus);
    i2c_slave_install_();
    return;
  }

  uint8_t response[2];
  err = i2c_master_transmit_receive(dev, data, len, response, sizeof(response), 200);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "I2C master write/read failed: %s", esp_err_to_name(err));
  } else {
    ESP_LOGD(TAG, "I2C RX (ack from spa): %s", format_hex_pretty(response, sizeof(response)).c_str());
  }

  i2c_master_bus_rm_device(dev);
  i2c_del_master_bus(bus);

  i2c_slave_install_();  // resume listening
#endif
}

#ifdef USE_ESP_IDF
// --- Transport B: I2C slave (listening) -------------------------------
// Active whenever we are not mid-send. Mirrors the Arduino bridge's
// Wire.begin(SPA_ADDRESS) + onReceive/onRequest. Uses I2C Slave v2
// (CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2 must be set in YAML sdkconfig_options) -
// v1 cannot report how many bytes were actually received, which this
// variable-length protocol depends on.

bool IRAM_ATTR GeckoSpa::i2c_slave_on_receive_cb(i2c_slave_dev_handle_t handle,
                                                  const i2c_slave_rx_done_event_data_t *edata,
                                                  void *user_data) {
  GeckoSpa *self = static_cast<GeckoSpa *>(user_data);
  BaseType_t hp_task_woken = pdFALSE;

  // edata->buffer is only valid for the duration of this callback under v2,
  // so copy out immediately rather than queuing a pointer to it.
  uint32_t len = edata->length;
  if (len > sizeof(self->i2c_rx_buf_))
    len = sizeof(self->i2c_rx_buf_);
  memcpy(self->i2c_rx_buf_, edata->buffer, len);

  xQueueSendFromISR(self->i2c_rx_queue_, &len, &hp_task_woken);
  return hp_task_woken == pdTRUE;
}

bool IRAM_ATTR GeckoSpa::i2c_slave_on_request_cb(i2c_slave_dev_handle_t handle,
                                                  const i2c_slave_request_event_data_t *edata,
                                                  void *user_data) {
  // Fires when the spa tries to read from us and nothing is queued yet -
  // push the 2-byte ack on demand (mirrors the Arduino bridge's requestEvent(),
  // which always answered reads with two zero bytes).
  uint32_t write_len = 0;
  i2c_slave_write(handle, I2C_ACK_BYTES, sizeof(I2C_ACK_BYTES), &write_len, 0);

  GeckoSpa *self = static_cast<GeckoSpa *>(user_data);
  self->i2c_ack_sent_count_.fetch_add(1, std::memory_order_relaxed);
  return false;
}

void GeckoSpa::i2c_slave_install_() {
  i2c_slave_config_t cfg = {};
  cfg.i2c_port = I2C_PORT;
  cfg.sda_io_num = i2c_sda_pin_;
  cfg.scl_io_num = i2c_scl_pin_;
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.send_buf_depth = 256;
  cfg.receive_buf_depth = 512;  // must comfortably hold the largest single I2C write
  cfg.slave_addr = i2c_address_;
  cfg.addr_bit_len = I2C_ADDR_BIT_LEN_7;
  cfg.flags.enable_internal_pullup = true;

  esp_err_t err = i2c_new_slave_device(&cfg, &i2c_slave_handle_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2c_new_slave_device failed: %s", esp_err_to_name(err));
    i2c_slave_handle_ = nullptr;
    return;
  }

  if (i2c_rx_queue_ == nullptr) {
    i2c_rx_queue_ = xQueueCreate(4, sizeof(uint32_t));
  }

  i2c_slave_event_callbacks_t cbs = {};
  cbs.on_receive = i2c_slave_on_receive_cb;
  cbs.on_request = i2c_slave_on_request_cb;
  i2c_slave_register_event_callbacks(i2c_slave_handle_, &cbs, this);

  ESP_LOGD(TAG, "I2C slave listening on addr 0x%02X", i2c_address_);
}

void GeckoSpa::i2c_slave_uninstall_() {
  if (i2c_slave_handle_ == nullptr)
    return;
  i2c_del_slave_device(i2c_slave_handle_);
  i2c_slave_handle_ = nullptr;
}
#endif  // USE_ESP_IDF

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
    arduino_ready_ = true;
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
    reset_retry_count_ = 0;
    if (connected_sensor_)
      connected_sensor_->publish_state(true);
    ESP_LOGI(TAG, "Spa connected (I2C traffic detected)");
  }

  // Log standalone messages as FULL-RX (not continuation parts of multi-part messages)
  // Continuation flag is byte[9]: 0x01 = more coming
  bool is_continuation = (len >= 10 && data[9] == 0x01);
  if (!is_continuation && msg_buffer_len_ == 0 && len > 0) {
    // Split into 32 bytes per line (64 hex characters)
    const int CHUNK_BYTES = 32;
    char hex_str[68];
    ESP_LOGI(TAG, "FULL-RX:%d bytes", len);
    for (int offset = 0; offset < len; offset += CHUNK_BYTES) {
      int chunk_len = (len - offset < CHUNK_BYTES) ? (len - offset) : CHUNK_BYTES;
      int pos = 0;
      for (int i = 0; i < chunk_len; i++) {
        pos += sprintf(hex_str + pos, "%02X", data[offset + i]);
      }
      ESP_LOGI(TAG, "  %03d: %s", offset, hex_str);
    }
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

  // 33-byte config file message - parse XML filename and send acknowledgment
  if (len == 33) {
    // Extract XML filename from bytes 16-28 (null-terminated string)
    char xml_name[16];
    int pos = 0;
    for (int i = 16; i < 29 && data[i] != 0 && pos < 15; i++) {
      xml_name[pos++] = (char)data[i];
    }
    xml_name[pos] = '\0';

    ESP_LOGI(TAG, "Handshake XML: %s", xml_name);

    // Parse version number from filename (e.g., inYT_C82.xml -> 82)
    // Look for _C or _S followed by digits
    const char *ver_ptr = strstr(xml_name, "_C");
    if (ver_ptr == nullptr) ver_ptr = strstr(xml_name, "_S");
    if (ver_ptr != nullptr) {
      int version = atoi(ver_ptr + 2);

      // Publish to appropriate sensor and store version
      if (strstr(xml_name, "_C") != nullptr) {
        config_version_ = version;
        if (config_version_sensor_)
          config_version_sensor_->publish_state(xml_name);
        ESP_LOGI(TAG, "Config version: %d", config_version_);
      } else if (strstr(xml_name, "_S") != nullptr) {
        status_version_ = version;
        if (status_version_sensor_)
          status_version_sensor_->publish_state(xml_name);
        ESP_LOGI(TAG, "Status version: %d", status_version_);

        // Select appropriate offsets based on status version
        if (status_version_ <= 50) {
          log_offsets_ = &GECKO_LOG_OFFSETS_V50;
          ESP_LOGI(TAG, "Using v50 log offsets");
        } else {
          log_offsets_ = &GECKO_LOG_OFFSETS_V51;
          ESP_LOGI(TAG, "Using v51+ log offsets");
        }
      }
    }

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
  // Header is 16 bytes: [0-13]=protocol header + [14-15]=5251("RQ" frame marker)
  // Stripping 16 bytes aligns payload with geckolib struct offsets
  // Only concatenate messages with byte[1]=0x09 (config/status type)
  static const int HEADER_LEN = 16;

  if (len >= HEADER_LEN && data[1] == 0x09) {
    bool more_coming = (data[9] == 0x01);

    // Add this part to buffer (strip 16-byte header including frame marker)
    int payload_start = HEADER_LEN;
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
    // Split into 32 bytes per line (64 hex characters)
    const int CHUNK_BYTES = 32;
    char hex_str[68];
    int total_bytes = msg_buffer_len_;
    ESP_LOGI(TAG, "FULL-RX:%d bytes", total_bytes);
    for (int offset = 0; offset < total_bytes; offset += CHUNK_BYTES) {
      int chunk_len = (total_bytes - offset < CHUNK_BYTES) ? (total_bytes - offset) : CHUNK_BYTES;
      int pos = 0;
      for (int i = 0; i < chunk_len; i++) {
        pos += sprintf(hex_str + pos, "%02X", msg_buffer_[offset + i]);
      }
      ESP_LOGI(TAG, "  %03d: %s", offset, hex_str);
    }

    // Check message type by size
    // ~162 bytes = status-only message (3 parts: 78+78+54 - 3*16 headers)
    // ~390 bytes = config+status message (8 parts with log section starting at offset 230)

    // We expect status message to be around 150 bytes long, but it varies per spa pack model
    // and software version.  The length will be consistent per pack, so once we've detected
    // it we can assume that all future messages at that length are status messages.  Assume
    // that anything between MIN_STATUS_MSG_LEN and 170 bytes long, with a 0 byte at offset 1,
    // is a status message if we don't yet know the correct length.
    if (status_msg_len_ == 0) {
      if ((msg_buffer_len_ >= MIN_STATUS_MSG_LEN) &&
          (msg_buffer_len_ <= 170) &&
          (msg_buffer_[1] == 0x00)) {
        ESP_LOGI(TAG, "Auto-detect %d as the standard status message length", msg_buffer_len_);
        status_msg_len_ = msg_buffer_len_;
      }
    }

    // Check message type by size
    // 162 bytes = status-only message (3 parts: 78+78+54 - 3*16 headers)
    // ~390 bytes = config+status message (8 parts with log section starting at offset 230)
    if ((msg_buffer_len_ == status_msg_len_) &&
        (status_msg_len_ != 0) &&
        (msg_buffer_[1] == 0x00)) {
      // Status-only message (162 bytes)
      ESP_LOGI(TAG, "Status msg (%db): [3]=%02X [5]=%02X [21-24]=%02X%02X%02X%02X [53]=%02X",
               msg_buffer_len_,
               msg_buffer_[3], msg_buffer_[5],
               msg_buffer_[21], msg_buffer_[22], msg_buffer_[23], msg_buffer_[24], msg_buffer_[53]);
      parse_status_message(msg_buffer_);
    } else if (msg_buffer_len_ >= 300 && msg_buffer_len_ <= 400) {
      // Config+status message (~390 bytes)
      // Config section has +2 byte offset (geckolib offset N → message byte N+2)
      static const int CFG_OFFSET = 2;  // Config struct offset

      // Parse config section (with +2 offset from geckolib struct definitions)
      // Geckolib offsets → message bytes: N → N+2
      uint8_t config_num = msg_buffer_[CFG_OFFSET + 0];
      uint16_t setpoint_raw = (msg_buffer_[CFG_OFFSET + 1] << 8) | msg_buffer_[CFG_OFFSET + 2];
      float setpoint_c = setpoint_raw / 18.0f;
      uint8_t filt_freq = msg_buffer_[CFG_OFFSET + 3];
      uint8_t temp_units = msg_buffer_[CFG_OFFSET + 33];  // 0=F, 1=C
      uint8_t time_format = msg_buffer_[CFG_OFFSET + 34]; // 0=NA, 1=AmPm, 2=24h
      uint8_t pump_timeout = msg_buffer_[CFG_OFFSET + 54];
      uint8_t light_timeout = msg_buffer_[CFG_OFFSET + 55];
      uint8_t econ_type = msg_buffer_[CFG_OFFSET + 70];   // 0=Standard, 1=Night
      uint8_t customer_id = msg_buffer_[CFG_OFFSET + 111];
      uint8_t num_zones = msg_buffer_[CFG_OFFSET + 127];
      uint8_t silent_mode = msg_buffer_[CFG_OFFSET + 157]; // 0=NA, 1=OFF, 2=ECONOMY, 3=SLEEP, 4=NIGHT

      static const char* time_fmt_str[] = {"NA", "AmPm", "24h"};
      static const char* silent_str[] = {"NA", "OFF", "ECONOMY", "SLEEP", "NIGHT"};
      static const char* econ_str[] = {"Standard", "Night"};

      ESP_LOGI(TAG, "Config: Ver=%d Setpoint=%.1f%s FiltFreq=%d TimeFormat=%s",
               config_num, setpoint_c, temp_units == 1 ? "C" : "F", filt_freq,
               time_format < 3 ? time_fmt_str[time_format] : "?");
      ESP_LOGI(TAG, "Config: PumpTimeout=%dmin LightTimeout=%dmin EconType=%s",
               pump_timeout, light_timeout,
               econ_type < 2 ? econ_str[econ_type] : "?");
      ESP_LOGI(TAG, "Config: CustomerID=%d Zones=%d SilentMode=%s",
               customer_id, num_zones,
               silent_mode < 5 ? silent_str[silent_mode] : "?");

      // Reuse the status parser on the status portion, if we know what the length of the
      // status message should be.
      if (status_msg_len_ != 0) {
        // Status portion is the very end of the message
        int STATUS_OFFSET = msg_buffer_len_ - status_msg_len_;
        // Check for some expected byte markers
        if ((msg_buffer_[STATUS_OFFSET - 1] == 0x3B) &&
            (msg_buffer_[STATUS_OFFSET + 1] == 0)) {
          parse_status_message(&msg_buffer_[STATUS_OFFSET]);
        }
      }
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
  // Convert geckolib offset to message byte: byte = geckolib_offset - 254
  // This accounts for the +2 byte misalignment between geckolib structs and actual message
  auto toB = [](uint16_t geckoOffset) -> uint16_t { return geckoOffset - 254; };

  // Get offsets from version-specific struct
  const GeckoLogOffsets &off = *log_offsets_;

  // Calculate message byte positions
  uint16_t b_hours = toB(off.hours);
  uint16_t b_quietState = toB(off.quietState);
  uint16_t b_udP1 = toB(off.udP1);
  uint16_t b_deviceStatus = toB(off.deviceStatus);
  uint16_t b_p1 = toB(off.p1);
  uint16_t b_udLi = toB(off.udLi);
  uint16_t b_realSetPoint = toB(off.realSetPointG);
  uint16_t b_displayedTemp = toB(off.displayedTempG);
  uint16_t b_lockMode = toB(off.lockMode);
  uint16_t b_packType = toB(off.packType);
  uint16_t b_udPumpTime = toB(off.udPumpTime);

  // === Decode all fields from geckolib-compatible offsets ===

  // Hours counter
  uint8_t hours = data[b_hours];

  // QuietState: 0=NOT_SET, 1=DRAIN, 2=SOAK, 3=OFF
  uint8_t quietState = data[b_quietState];
  static const char* quiet_str[] = {"NOT_SET", "DRAIN", "SOAK", "OFF"};

  // User demand P1-P4 (2-bit fields in UdP1 byte)
  user_demand_state_ = data[b_udP1];
  uint8_t udP1 = (user_demand_state_ >> 0) & 0x03;  // bits 0-1
  uint8_t udP2 = (user_demand_state_ >> 2) & 0x03;  // bits 2-3
  uint8_t udP3 = (user_demand_state_ >> 4) & 0x03;  // bits 4-5
  uint8_t udP4 = (user_demand_state_ >> 6) & 0x03;  // bits 6-7
  static const char* pump_ud_str[] = {"OFF", "LO", "HI", "?"};

  // Device status byte (CP, BL, Heater, Waterfall)
  uint8_t devStatus = data[b_deviceStatus];
  bool cp_on = (devStatus >> 2) & 0x01;      // bit 2: CP (circulation pump)
  bool bl_on = (devStatus >> 1) & 0x01;      // bit 1: BL (blower)
  bool heater_on = (devStatus >> 5) & 0x01;  // bit 5: MSTR_HEATER
  bool waterfall = (devStatus >> 7) & 0x01;  // bit 7: Waterfall

  // P1-P4 device status (2-bit fields)
  uint8_t p1_raw = data[b_p1];
  uint8_t p1_state = (p1_raw >> 0) & 0x03;  // bits 0-1: 0=OFF, 1=HIGH, 2=LOW
  uint8_t p2_state = (p1_raw >> 2) & 0x03;  // bits 2-3
  uint8_t p3_state = (p1_raw >> 4) & 0x03;  // bits 4-5
  uint8_t p4_state = (p1_raw >> 6) & 0x03;  // bits 6-7
  static const char* pump_state_str[] = {"OFF", "HIGH", "LOW", "?"};

  // Light user demand
  uint8_t udLi = data[b_udLi];

  // Lock mode: 0=UNLOCK, 1=PARTIAL, 2=FULL
  uint8_t lockMode = data[b_lockMode];
  static const char* lock_str[] = {"UNLOCK", "PARTIAL", "FULL"};

  // Pack type
  uint8_t packType = data[b_packType];
  static const char* pack_str[] = {"Unknown", "inXE", "MasIBC", "MIA", "DJS4", "inClear", "inXM", "K600", "inTerface", "inTouch", "inYT"};

  // Pump timer countdown
  uint8_t pumpTime = data[b_udPumpTime];

  // Temperature (word values, big-endian)
  uint16_t target_raw = (data[b_realSetPoint] << 8) | data[b_realSetPoint + 1];
  uint16_t actual_raw = (data[b_displayedTemp] << 8) | data[b_displayedTemp + 1];
  float target_temp = target_raw / 18.0f;
  float actual_temp = actual_raw / 18.0f;

  // === Log decoded status (geckolib format) ===
  ESP_LOGI(TAG, "Status[v%d]: Hours=%d QuietState=%s LockMode=%s PackType=%s",
           status_version_, hours,
           quietState < 4 ? quiet_str[quietState] : "?",
           lockMode < 3 ? lock_str[lockMode] : "?",
           packType < 11 ? pack_str[packType] : "?");

  ESP_LOGI(TAG, "Status: Temp=%.1f/%.1f°C Heater=%s CP=%s BL=%s Waterfall=%s",
           target_temp, actual_temp,
           heater_on ? "ON" : "OFF",
           cp_on ? "ON" : "OFF",
           bl_on ? "ON" : "OFF",
           waterfall ? "ON" : "OFF");

  ESP_LOGI(TAG, "Status: P1=%s P2=%s P3=%s P4=%s PumpTimer=%dmin",
           pump_state_str[p1_state], pump_state_str[p2_state],
           pump_state_str[p3_state], pump_state_str[p4_state],
           pumpTime);

  ESP_LOGI(TAG, "Status: UdP1=%s UdP2=%s UdP3=%s UdP4=%s UdLi=%s",
           pump_ud_str[udP1], pump_ud_str[udP2],
           pump_ud_str[udP3], pump_ud_str[udP4],
           udLi ? "ON" : "OFF");

  // === Update internal state and entities ===

  // Derive states for Home Assistant entities
  bool new_standby = (quietState == 0x03);  // OFF in QuietState means standby
  bool new_circ = cp_on;              // Circulation pump from device status (CP)
  bool new_waterfall = waterfall;     // Waterfall from device status
  bool new_blower = bl_on;            // Blower from device status (BL)
  bool new_heating = heater_on;       // Heater from device status
  bool new_light = (udLi != 0);       // Light from UdLi (user demand light)

  // P1-P4 individual pump states (0=OFF, 1=HIGH, 2=LOW)
  uint8_t new_p1 = p1_state;
  uint8_t new_p2 = p2_state;
  uint8_t new_p3 = p3_state;
  uint8_t new_p4 = p4_state;

  float new_target = target_temp;
  float new_actual = actual_temp;
  bool temp_valid = (target_raw != 0 || actual_raw != 0);

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

  if (first || new_circ != circ_state_) {
    circ_state_ = new_circ;
    ESP_LOGI(TAG, "Circulation: %s", circ_state_ ? "ON" : "OFF");
    if (circ_switch_)
      circ_switch_->publish_state(circ_state_);
  }

  if (first || new_waterfall != waterfall_state_) {
    waterfall_state_ = new_waterfall;
    ESP_LOGI(TAG, "Waterfall: %s", waterfall_state_ ? "ON" : "OFF");
    if (waterfall_sensor_)
      waterfall_sensor_->publish_state(waterfall_state_);
  }

  if (first || new_blower != blower_state_) {
    blower_state_ = new_blower;
    ESP_LOGI(TAG, "Blower: %s", blower_state_ ? "ON" : "OFF");
    if (blower_sensor_)
      blower_sensor_->publish_state(blower_state_);
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

  // Update LockMode sensor
  if (first || lockMode != lock_mode_) {
    lock_mode_ = lockMode;
    if (lock_mode_sensor_) {
      lock_mode_sensor_->publish_state(lockMode < 3 ? lock_str[lockMode] : "?");
    }
  }

  // Update PackType sensor
  if (first || packType != pack_type_) {
    pack_type_ = packType;
    if (pack_type_sensor_) {
      pack_type_sensor_->publish_state(packType < 11 ? pack_str[packType] : "?");
    }
  }

  // Update PumpTimer sensor
  if (first || pumpTime != pump_timer_) {
    pump_timer_ = pumpTime;
    if (pump_timer_sensor_) {
      pump_timer_sensor_->publish_state(pumpTime);
    }
  }

  // Only update temperature if valid data was received
  if (temp_valid && (first || abs(new_target - target_temp_) > 0.1 || abs(new_actual - actual_temp_) > 0.1)) {
    target_temp_ = new_target;
    actual_temp_ = new_actual;
    ESP_LOGI(TAG, "Temp: target=%.1f actual=%.1f", target_temp_, actual_temp_);
    update_climate_state();
  }

  // Update P1-P4 pump states (all controllable switches)
  if (first || new_p1 != pump1_state_) {
    pump1_state_ = new_p1;
    if (pump1_switch_)
      pump1_switch_->publish_state(pump1_state_ != 0);
  }
  if (first || new_p2 != pump2_state_) {
    pump2_state_ = new_p2;
    if (pump2_switch_)
      pump2_switch_->publish_state(pump2_state_ != 0);
  }
  if (first || new_p3 != pump3_state_) {
    pump3_state_ = new_p3;
    if (pump3_switch_)
      pump3_switch_->publish_state(pump3_state_ != 0);
  }
  if (first || new_p4 != pump4_state_) {
    pump4_state_ = new_p4;
    if (pump4_switch_)
      pump4_switch_->publish_state(pump4_state_ != 0);
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
  // heating_state_ is the authoritative source for whether heater is running
  if (heating_state_) {
    climate_->action = climate::CLIMATE_ACTION_HEATING;
  } else if (target_temp_ < actual_temp_) {
    // Spa is cooling down (no active cooling, just natural heat loss)
    climate_->action = climate::CLIMATE_ACTION_COOLING;
  } else {
    climate_->action = climate::CLIMATE_ACTION_IDLE;
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
    uint8_t reset_day, reset_month, reset_year;
    if (notif_date_format_ == NotifDateFormat::D_M_Y) {
      reset_day = data[offset + 1];
      reset_month = data[offset + 2];
      reset_year = data[offset + 3];  // 2-digit year
    } else {
      reset_year = data[offset + 1];  // 2-digit year
      reset_month = data[offset + 2];
      reset_day = data[offset + 3];
    }
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

    // Publish state if it has changed
    if (strcmp(date_str, notification_date_[id - 1]))
    {
      ESP_LOGI(TAG, "Publish changed notification %d : %s", id, date_str);
      if (sensor) {
        sensor->publish_state(date_str);
      }
      strcpy(notification_date_[id - 1], date_str);
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
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE | climate::CLIMATE_SUPPORTS_ACTION);
  traits.set_supported_modes({climate::CLIMATE_MODE_HEAT, climate::CLIMATE_MODE_COOL});
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
  } else if (switch_type_ == "circulation") {
    parent_->send_circ_command(state);
  } else if (switch_type_ == "pump1") {
    parent_->send_pump1_command(state ? 1 : 0);  // 1=HIGH, 0=OFF
  } else if (switch_type_ == "pump2") {
    parent_->send_pump2_command(state ? 1 : 0);  // EXPERIMENTAL
  } else if (switch_type_ == "pump3") {
    parent_->send_pump3_command(state ? 1 : 0);  // EXPERIMENTAL
  } else if (switch_type_ == "pump4") {
    parent_->send_pump4_command(state ? 1 : 0);  // EXPERIMENTAL
  }
  // State will be published when spa confirms the change
}

// GeckoSpaSelect implementation
void GeckoSpaSelect::setup() {
  // Initialize preferences
  this->pref_ = global_preferences->make_preference<uint8_t>(this->get_object_id_hash());

  // Restore saved state on boot
  if (this->pref_.load(&this->saved_index_) && this->saved_index_ < 5) {
    static const char *prog_names[] = {"Away", "Standard", "Energy", "Super Energy", "Weekend"};
    this->publish_state(prog_names[this->saved_index_]);
    ESP_LOGI("gecko_spa", "Restored program state: %s (index %d)", prog_names[this->saved_index_], this->saved_index_);
  }
}

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

  // Save the state
  this->saved_index_ = prog;
  this->pref_.save(&this->saved_index_);

  parent_->send_program_command(prog);
  this->publish_state(value);
}

}  // namespace gecko_spa
}  // namespace esphome
