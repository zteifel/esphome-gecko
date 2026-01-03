#pragma once

#include <string>
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace gecko_spa {

class GeckoSpaClimate;

class GeckoSpa : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Entity setters
  void set_light_switch(switch_::Switch *sw) { light_switch_ = sw; }
  void set_pump_switch(switch_::Switch *sw) { pump_switch_ = sw; }
  void set_circ_switch(switch_::Switch *sw) { circ_switch_ = sw; }
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
  void set_reset_pin(GPIOPin *pin) { reset_pin_ = pin; }

  // Command methods
  void send_light_command(bool on);
  void send_pump_command(bool on);
  void send_circ_command(bool on);
  void send_program_command(uint8_t prog);
  void send_temperature_command(float temp_c);
  void request_status();
  void reset_arduino();

  // State getters
  bool get_light_state() { return light_state_; }
  bool get_pump_state() { return pump_state_; }
  bool get_circ_state() { return circ_state_; }
  float get_target_temp() { return target_temp_; }
  float get_actual_temp() { return actual_temp_; }
  bool is_heating() { return heating_state_; }

 protected:
  // Entity pointers
  switch_::Switch *light_switch_{nullptr};
  switch_::Switch *pump_switch_{nullptr};
  switch_::Switch *circ_switch_{nullptr};
  select::Select *program_select_{nullptr};
  binary_sensor::BinarySensor *standby_sensor_{nullptr};
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  climate::Climate *climate_{nullptr};
  text_sensor::TextSensor *rinse_filter_sensor_{nullptr};
  text_sensor::TextSensor *clean_filter_sensor_{nullptr};
  text_sensor::TextSensor *change_water_sensor_{nullptr};
  text_sensor::TextSensor *spa_checkup_sensor_{nullptr};
  text_sensor::TextSensor *spa_time_sensor_{nullptr};
  GPIOPin *reset_pin_{nullptr};

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
  uint32_t last_i2c_time_{0};
  uint32_t last_go_send_time_{0};
  uint32_t reset_start_time_{0};
  bool reset_in_progress_{false};

  // UART buffer
  char uart_buffer_[512];
  uint16_t uart_pos_{0};

  // Multi-part message buffer (byte[10]=0x01 means more coming, 0x00 means last)
  uint8_t msg_buffer_[512];
  uint16_t msg_buffer_len_{0};

  // GO keep-alive message
  static const uint8_t GO_MESSAGE[15];

  uint8_t calc_checksum(const uint8_t *data, uint8_t len);
  void send_i2c_message(const uint8_t *data, uint8_t len);
  uint8_t hex_to_byte(char high, char low);
  void process_proxy_message(const char *msg);
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

  void control(const std::string &value) override;

 protected:
  GeckoSpa *parent_{nullptr};
};

}  // namespace gecko_spa
}  // namespace esphome
