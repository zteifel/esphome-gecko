#pragma once

#include <cstdint>
#include <string>
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/sensor/sensor.h"

// Two mutually exclusive transports to the spa's I2C bus:
//   - UART: bridged through an Arduino acting as a "dumb" I2C proxy (the
//     original design - see uart_ below). Only usable if uart_id: was
//     actually configured - __init__.py defines USE_GECKO_SPA_UART only in
//     that case, since ESPHome only copies a component's source files (here,
//     uart's own headers) into the build when that component appears in the
//     config at all, and uart_id: is optional.
//   - Direct I2C: the ESP32 owns the I2C port itself via a raw ESP-IDF driver,
//     toggling between slave and master roles - esphome::i2c doesn't support
//     that role-swap, and the driver calls involved aren't available under
//     framework: arduino, so this whole path is compiled out unless
//     USE_ESP_IDF is defined (i.e. framework: esp-idf).
#ifdef USE_GECKO_SPA_UART
#include "esphome/components/uart/uart.h"
#endif
#ifdef USE_ESP_IDF
#include "driver/i2c_slave.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <atomic>
#endif

namespace esphome {
namespace gecko_spa {

// Geckolib-compatible offset structure for log/status messages
// Offsets are geckolib offsets (base 256), convert to message byte with: byte = offset - 254
struct GeckoLogOffsets {
  uint16_t hours;           // Operating hours
  uint16_t quietState;      // Quiet/drain/soak mode
  uint16_t udP1;            // User demand P1-P4 (2-bit fields)
  uint16_t deviceStatus;    // CP, BL, Heater, Waterfall bits
  uint16_t p1;              // P1-P4 device status (2-bit fields)
  uint16_t udLi;            // Light user demand
  uint16_t realSetPointG;   // Target temperature (word)
  uint16_t displayedTempG;  // Actual temperature (word)
  uint16_t lockMode;        // Keypad lock status
  uint16_t packType;        // Pack type identifier
  uint16_t udPumpTime;      // Pump timer countdown
};

// Default offsets for inYT v51+ (most common)
static const GeckoLogOffsets GECKO_LOG_OFFSETS_V51 = {
  .hours = 256,
  .quietState = 257,
  .udP1 = 259,
  .deviceStatus = 260,
  .p1 = 261,
  .udLi = 307,
  .realSetPointG = 275,
  .displayedTempG = 277,
  .lockMode = 310,
  .packType = 289,
  .udPumpTime = 303,
};

// Offsets for inYT v50 (older version with shifted offsets)
static const GeckoLogOffsets GECKO_LOG_OFFSETS_V50 = {
  .hours = 284,
  .quietState = 285,
  .udP1 = 258,
  .deviceStatus = 259,
  .p1 = 260,
  .udLi = 307,
  .realSetPointG = 274,
  .displayedTempG = 276,
  .lockMode = 309,
  .packType = 288,
  .udPumpTime = 302,
};

class GeckoSpaClimate;

enum class NotifDateFormat : uint8_t {
  Y_M_D = 0,
  D_M_Y = 1
};

class GeckoSpa : public Component {
 public:
  void setup() override;
  void loop() override;
  void on_shutdown() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Transport A: Arduino I2C-proxy over UART. Composition rather than
  // inheriting uart::UARTDevice, since exactly one of the two transports is
  // active for a given device and the other must add zero footprint.
#ifdef USE_GECKO_SPA_UART
  void set_uart_parent(uart::UARTComponent *parent) { uart_ = parent; }
#endif
  bool has_uart() const { return uart_ != nullptr; }

#ifdef USE_ESP_IDF
  // Transport B: direct I2C (raw ESP-IDF port, not esphome::i2c - see
  // includes above). Only usable under framework: esp-idf; enforced in
  // __init__.py's config validation.
  void set_i2c_sda_pin(int pin) { i2c_sda_pin_ = static_cast<gpio_num_t>(pin); }
  void set_i2c_scl_pin(int pin) { i2c_scl_pin_ = static_cast<gpio_num_t>(pin); }
  void set_i2c_address(uint8_t address) { i2c_address_ = address; }
#endif

  // Entity setters - switches (controllable)
  void set_light_switch(switch_::Switch *sw) { light_switch_ = sw; }
  void set_circ_switch(switch_::Switch *sw) { circ_switch_ = sw; }
  void set_pump1_switch(switch_::Switch *sw) { pump1_switch_ = sw; }
  void set_pump2_switch(switch_::Switch *sw) { pump2_switch_ = sw; }
  void set_pump3_switch(switch_::Switch *sw) { pump3_switch_ = sw; }
  void set_pump4_switch(switch_::Switch *sw) { pump4_switch_ = sw; }
  // Entity setters - binary sensors (read-only status)
  void set_waterfall_sensor(binary_sensor::BinarySensor *bs) { waterfall_sensor_ = bs; }
  void set_blower_sensor(binary_sensor::BinarySensor *bs) { blower_sensor_ = bs; }
  void set_program_select(select::Select *sel) { program_select_ = sel; }
  void set_standby_sensor(binary_sensor::BinarySensor *bs) {
    standby_sensor_ = bs;
    bs->publish_state(standby_state_);
  }
  void set_connected_sensor(binary_sensor::BinarySensor *bs) {
    connected_sensor_ = bs;
    bs->publish_state(connected_);
  }
  void set_climate(climate::Climate *cl) { climate_ = cl; }
  void set_rinse_filter_sensor(text_sensor::TextSensor *s) { rinse_filter_sensor_ = s; }
  void set_clean_filter_sensor(text_sensor::TextSensor *s) { clean_filter_sensor_ = s; }
  void set_change_water_sensor(text_sensor::TextSensor *s) { change_water_sensor_ = s; }
  void set_spa_checkup_sensor(text_sensor::TextSensor *s) { spa_checkup_sensor_ = s; }
  void set_spa_time_sensor(text_sensor::TextSensor *s) { spa_time_sensor_ = s; }
  void set_config_version_sensor(text_sensor::TextSensor *s) { config_version_sensor_ = s; }
  void set_status_version_sensor(text_sensor::TextSensor *s) { status_version_sensor_ = s; }
  void set_lock_mode_sensor(text_sensor::TextSensor *s) { lock_mode_sensor_ = s; }
  void set_pack_type_sensor(text_sensor::TextSensor *s) { pack_type_sensor_ = s; }
  void set_pump_timer_sensor(sensor::Sensor *s) { pump_timer_sensor_ = s; }
  void set_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }
  void set_notif_date_format(NotifDateFormat format) { notif_date_format_ = format; }

  // Command methods
  void send_light_command(bool on);
  void send_circ_command(bool on);
  void send_pump1_command(uint8_t state);  // state: 0=OFF, 1=HIGH, 2=LOW
  void send_pump2_command(uint8_t state);  // Experimental: func ID 0x04
  void send_pump3_command(uint8_t state);  // Experimental: func ID 0x05
  void send_pump4_command(uint8_t state);  // Experimental: func ID 0x06
  void send_program_command(uint8_t prog);
  void send_temperature_command(float temp_c);
  void request_status();
  void reset_arduino();  // Only meaningful for the UART transport - no-op without reset_pin_

  // State getters
  bool get_light_state() { return light_state_; }
  bool get_circ_state() { return circ_state_; }
  bool get_waterfall_state() { return waterfall_state_; }
  uint8_t get_pump1_state() { return pump1_state_; }  // 0=OFF, 1=HIGH, 2=LOW
  uint8_t get_pump2_state() { return pump2_state_; }
  uint8_t get_pump3_state() { return pump3_state_; }
  uint8_t get_pump4_state() { return pump4_state_; }
  float get_target_temp() { return target_temp_; }
  float get_actual_temp() { return actual_temp_; }
  bool is_heating() { return heating_state_; }

 protected:
  // Entity pointers - switches (controllable)
  switch_::Switch *light_switch_{nullptr};
  switch_::Switch *circ_switch_{nullptr};
  switch_::Switch *pump1_switch_{nullptr};
  switch_::Switch *pump2_switch_{nullptr};
  switch_::Switch *pump3_switch_{nullptr};
  switch_::Switch *pump4_switch_{nullptr};
  // Entity pointers - binary sensors (read-only)
  binary_sensor::BinarySensor *waterfall_sensor_{nullptr};
  binary_sensor::BinarySensor *blower_sensor_{nullptr};
  select::Select *program_select_{nullptr};
  binary_sensor::BinarySensor *standby_sensor_{nullptr};
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  climate::Climate *climate_{nullptr};
  text_sensor::TextSensor *rinse_filter_sensor_{nullptr};
  text_sensor::TextSensor *clean_filter_sensor_{nullptr};
  text_sensor::TextSensor *change_water_sensor_{nullptr};
  text_sensor::TextSensor *spa_checkup_sensor_{nullptr};
  text_sensor::TextSensor *spa_time_sensor_{nullptr};
  text_sensor::TextSensor *config_version_sensor_{nullptr};
  text_sensor::TextSensor *status_version_sensor_{nullptr};
  text_sensor::TextSensor *lock_mode_sensor_{nullptr};
  text_sensor::TextSensor *pack_type_sensor_{nullptr};
  sensor::Sensor *pump_timer_sensor_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  NotifDateFormat notif_date_format_{NotifDateFormat::D_M_Y};

  // State
  bool light_state_{false};
  bool circ_state_{false};
  bool waterfall_state_{false};
  bool blower_state_{false};
  bool heating_state_{false};
  bool standby_state_{false};
  bool connected_{false};
  bool first_status_received_{false};
  uint8_t user_demand_state_{0};  // Bitfield from udP1-udP4 (P1-P4 user demand)
  uint8_t pump1_state_{0};   // 0=OFF, 1=HIGH, 2=LOW
  uint8_t pump2_state_{0};   // Read-only
  uint8_t pump3_state_{0};   // Read-only
  uint8_t pump4_state_{0};   // Read-only
  uint8_t lock_mode_{0};
  uint8_t pack_type_{0};
  uint8_t pump_timer_{0};
  uint8_t program_id_{0xFF};
  float target_temp_{0};
  float actual_temp_{0};
  uint32_t last_i2c_time_{0};
  uint32_t last_go_send_time_{0};
  uint32_t reset_start_time_{0};
  bool reset_in_progress_{false};
  uint8_t reset_retry_count_{0};        // Number of consecutive resets without recovery
  bool arduino_ready_{false};           // True after Arduino sends READY
  char notification_date_[4][12]{ "", "", "", ""};

  // Version tracking (parsed from handshake XML filenames)
  uint8_t config_version_{0};   // e.g., 82 from inYT_C82.xml
  uint8_t status_version_{0};   // e.g., 81 from inYT_S81.xml
  const GeckoLogOffsets *log_offsets_{&GECKO_LOG_OFFSETS_V51};  // Default to v51+

  // --- Transport A: UART (Arduino I2C proxy) ---------------------------
  // Stored type-erased so this member (and the has_uart() null-check above)
  // compile even when USE_GECKO_SPA_UART isn't defined and uart::UARTComponent
  // isn't available - only the code that actually calls methods on it needs
  // to be behind that #ifdef (see gecko_spa.cpp).
  void *uart_{nullptr};
  char uart_buffer_[512];
  uint16_t uart_pos_{0};
  uint8_t hex_to_byte(char high, char low);
  void process_proxy_message(const char *msg);

#ifdef USE_ESP_IDF
  // --- Transport B: direct I2C (raw ESP-IDF port - see includes above) ---
  gpio_num_t i2c_sda_pin_{GPIO_NUM_NC};
  gpio_num_t i2c_scl_pin_{GPIO_NUM_NC};
  uint8_t i2c_address_{0x17};
  static const i2c_port_num_t I2C_PORT{I2C_NUM_0};

  // I2C slave state (I2C Slave v2 driver - requires CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2
  // set via sdkconfig_options in YAML). Active most of the time, listening for
  // unsolicited writes from the spa's own controller board.
  i2c_slave_dev_handle_t i2c_slave_handle_{nullptr};
  QueueHandle_t i2c_rx_queue_{nullptr};   // carries just the received length (uint32_t) -
                                          // the bytes themselves are copied into i2c_rx_buf_
                                          // by the ISR callback, since v2's callback buffer
                                          // is only valid for the duration of the callback
  uint8_t i2c_rx_buf_[256];
  static const uint8_t I2C_ACK_BYTES[2];  // {0x00, 0x00} - reply to the spa's follow-up read

  // Debug visibility into the on-request ack write, which happens in ISR context
  // (i2c_slave_on_request_cb) where ESP_LOG* is not safe to call. The ISR just
  // bumps this atomic counter; loop() compares it against i2c_ack_logged_count_
  // and logs the (fixed) ack bytes on the main task once it changes.
  std::atomic<uint32_t> i2c_ack_sent_count_{0};
  uint32_t i2c_ack_logged_count_{0};

  void i2c_slave_install_();
  void i2c_slave_uninstall_();
  static bool IRAM_ATTR i2c_slave_on_receive_cb(i2c_slave_dev_handle_t handle,
                                                 const i2c_slave_rx_done_event_data_t *edata,
                                                 void *user_data);
  static bool IRAM_ATTR i2c_slave_on_request_cb(i2c_slave_dev_handle_t handle,
                                                 const i2c_slave_request_event_data_t *edata,
                                                 void *user_data);
#endif

  // Multi-part message buffer (byte[10]=0x01 means more coming, 0x00 means last)
  uint8_t msg_buffer_[512];
  uint16_t msg_buffer_len_{0};

  // GO keep-alive message
  static const uint8_t GO_MESSAGE[15];

  // Autodetected status message length
  // Currently we don't parse beyond offset 112, so a min length of 120 is safe.
  uint16_t status_msg_len_{0};
  static const uint16_t MIN_STATUS_MSG_LEN{120};

  uint8_t calc_checksum(const uint8_t *data, uint8_t len);
  void send_i2c_message(const uint8_t *data, uint8_t len);
  void process_i2c_message(const uint8_t *data, uint8_t len);
  void parse_status_message(const uint8_t *data);
  void parse_notification_message(const uint8_t *data);
  int days_since_2000(int day, int month, int year);
  void update_climate_state();
};

class GeckoSpaClimate : public Component, public climate::Climate {
 public:
  GeckoSpaClimate(GeckoSpa *parent) : parent_(parent) {}

  void setup() override;
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

 protected:
  GeckoSpa *parent_;
};

class GeckoSpaSwitch : public Component, public switch_::Switch {
 public:
  void set_parent(GeckoSpa *parent) { parent_ = parent; }
  void set_switch_type(const std::string &type) { switch_type_ = type; }

  void write_state(bool state) override;

 protected:
  GeckoSpa *parent_{nullptr};
  std::string switch_type_;
};

class GeckoSpaSelect : public Component, public select::Select {
 public:
  void set_parent(GeckoSpa *parent) { parent_ = parent; }
  void setup() override;

  void control(const std::string &value) override;

 protected:
  GeckoSpa *parent_{nullptr};
  ESPPreferenceObject pref_;
  uint8_t saved_index_{0xFF};
};

}  // namespace gecko_spa
}  // namespace esphome
