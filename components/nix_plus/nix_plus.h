#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/time/real_time_clock.h"
#include "esphome/components/text/text.h"
#include "esphome/components/button/button.h"

#include <vector>
#include <string>
#include <deque>

namespace esphome {
namespace nix_plus {

class NixPlus;

struct CommandTransaction {
  uint8_t opcode{0};
  uint8_t data[64]{0};
  uint8_t expected_response{0};
  uint8_t attempts{0};
  uint8_t max_attempts{3};
  uint32_t last_send_ms{0};
  uint32_t timeout_ms{250};
};

class NixPlusLight : public light::LightOutput {
 public:
  void set_parent(NixPlus *parent) { parent_ = parent; }
  light::LightTraits get_traits() override;
  void write_state(light::LightState *state) override;

 protected:
  NixPlus *parent_{nullptr};
};

class NixPlusDisplayLight : public light::LightOutput {
 public:
  void set_parent(NixPlus *parent) { parent_ = parent; }
  light::LightTraits get_traits() override;
  void write_state(light::LightState *state) override;

 protected:
  NixPlus *parent_{nullptr};
};

class NixPlus : public Component, public uart::UARTDevice {
 public:
  NixPlus() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void queue_command(const uint8_t *data, size_t len = 64, uint8_t expected_response = 0, uint8_t max_attempts = 3);
  void process_tx_queue();
  void rollback_unconfirmed_state(const CommandTransaction &tx);

  void set_num_digits(uint8_t num_digits) {
    user_configured_digits_ = num_digits;
    if (num_digits > 0) {
      num_digits_ = num_digits;
    }
  }
  uint8_t get_num_digits() const { return num_digits_; }

  void set_time(time::RealTimeClock *time) { time_ = time; }
  void set_ambient_light_sensor(sensor::Sensor *s) { ambient_light_sensor_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { temperature_sensor_ = s; }
  void set_day_mode_sensor(binary_sensor::BinarySensor *s) { day_mode_sensor_ = s; }
  void set_night_mode_sensor(binary_sensor::BinarySensor *s) { night_mode_sensor_ = s; }
  void set_model_name_sensor(text_sensor::TextSensor *s) { model_name_sensor_ = s; }
  void set_ip_address_sensor(text_sensor::TextSensor *s) { ip_address_sensor_ = s; }
  std::string get_current_ip_str();
  void set_display_light_state(light::LightState *s);
  void set_backlight_light_state(light::LightState *s);

  void set_ambient_mode_switch(switch_::Switch *s) { ambient_mode_switch_ = s; }
  void set_sntp_switch(switch_::Switch *s) { sntp_switch_ = s; }
  void set_time_zone_text(text::Text *t) { time_zone_text_ = t; }
  void set_detect_timezone_button(button::Button *b) { detect_timezone_button_ = b; }
  bool is_handshake_completed() const { return handshake_completed_; }

  // Actions & Control (Existing clock commands)
  void display_number(float value, uint8_t duration_sec = 5);
  void display_value(const std::string &value_str, uint8_t duration_sec = 5);
  void clear_display_override();
  void show_date_screen(uint8_t duration_sec = 5);
  void show_temperature_screen(uint8_t duration_sec = 5);
  void show_demo_screen();
  void set_display_light(bool is_on, float brightness);
  void set_display_brightness(uint8_t level); // 0 (min) to 7 (100%)
  void set_display_power(bool on);
  void set_backlight(bool is_on, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
  void set_rgb_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 255);
  void set_backlight_cycling(uint8_t mode);
  void revert_lights();
  void schedule_state_confirmation();
  void play_tone(uint16_t frequency_hz, uint16_t duration_ms, uint8_t volume = 7);
  void beep(uint8_t count = 1);
  void sync_time_to_clock();
  void sync_time_to_clock(ESPTime time);
  void trigger_time_sync();

  // Timer Controls (Opcode 0x20)
  void start_timer(uint32_t seconds);
  void start_timer(uint8_t hours, uint8_t minutes, uint8_t seconds);
  void stop_timer();
  void pause_timer();
  void resume_timer();

  // Periodic polling & handshakes
  void send_device_info_request(); // Opcode 0x10
  void read_clock_settings();      // Opcode 0x02
  void request_sensors();          // Opcode 0x11
  void request_module_status();    // Opcode 0xE0
  void enable_clock_module();      // Opcode 0xE1
  void send_packet(const uint8_t *data, size_t len = 64);

 protected:
  static uint16_t crc16_ccitt(const uint8_t *data, size_t len);
  void process_line(const std::string &line);
  void process_binary_frame(const uint8_t *data, size_t len);

  uint8_t user_configured_digits_{0}; // 0 = Auto-detect
  uint8_t num_digits_{4};            // Default fallback
  uint8_t model_type_{0};
  std::string model_name_{"Unknown"};
  uint16_t serial_number_{0};
  uint8_t hw_version_{0};
  uint8_t fw_major_{0};
  uint8_t fw_minor_{0};
  bool handshake_completed_{false};

  light::LightState *display_light_state_{nullptr};
  light::LightState *backlight_light_state_{nullptr};

  bool display_power_state_{true};
  bool display_initialized_{false};
  uint8_t display_brightness_level_{7};
  bool last_brightness_valid_{false};
  uint8_t last_brightness_level_{255};

  bool backlight_power_state_{false};
  bool backlight_initialized_{false};
  bool last_rgb_valid_{false};
  uint8_t last_g_val_{255};
  uint8_t last_r_val_{255};
  uint8_t last_b_val_{255};
  int8_t active_backlight_effect_{-1}; // -1 = solid colour, 0-5 = cycle modes

  time::RealTimeClock *time_{nullptr};
  switch_::Switch *sntp_switch_{nullptr};
  text::Text *time_zone_text_{nullptr};
  button::Button *detect_timezone_button_{nullptr};
  bool manual_time_set_{false};
  ESPTime manual_time_{};
  bool clock_settings_valid_{false};
  uint8_t clock_settings_[7]{0b00000001, 0b00010011, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint32_t last_settings_request_{0};
  sensor::Sensor *ambient_light_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  binary_sensor::BinarySensor *day_mode_sensor_{nullptr};
  binary_sensor::BinarySensor *night_mode_sensor_{nullptr};
  text_sensor::TextSensor *model_name_sensor_{nullptr};
  text_sensor::TextSensor *ip_address_sensor_{nullptr};
  uint32_t last_ip_publish_{0};
  std::string last_published_ip_{""};
  uint32_t last_rx_byte_ms_{0};

  std::vector<uint8_t> rx_buffer_;
  std::string line_buffer_;
  std::vector<uint8_t> improv_rx_buffer_;
  uint32_t last_improv_rx_ms_{0};
  bool in_improv_frame_{false};
  uint16_t improv_expected_len_{0};
  uint32_t last_sensor_request_{0};
  uint32_t last_handshake_request_{0};
  uint32_t last_info_poll_{0};
  uint32_t last_module_status_request_{0};
  uint32_t last_module_enable_attempt_{0};
  uint32_t last_ascii_rx_ms_{0};
  uint8_t clock_module_state_{0};
  uint8_t clock_module_mode_{0};
  bool initial_sync_done_{false};
  switch_::Switch *ambient_mode_switch_{nullptr};

  std::deque<CommandTransaction> tx_queue_;

  // Confirmed hardware states (for tracking & rollback if a command fails)
  uint8_t confirmed_display_level_{7};
  bool confirmed_display_power_{true};
  uint8_t confirmed_r_{0};
  uint8_t confirmed_g_{0};
  uint8_t confirmed_b_{0};
  bool confirmed_backlight_power_{false};
};

}  // namespace nix_plus
}  // namespace esphome
