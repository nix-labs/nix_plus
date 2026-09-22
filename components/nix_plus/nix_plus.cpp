#include "nix_plus.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include "esphome/core/preferences.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/components/network/util.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace esphome {

namespace improv_serial {
class ImprovSerialComponent {
 public:
  bool feed_byte(uint8_t byte);
};
extern ImprovSerialComponent *global_improv_serial_component;
}

namespace nix_plus {

static const char *const TAG = "nix_plus";

// CCITT CRC16 Table matching PIC32 DMACRC16 implementation
static const uint16_t CRC16_TABLE[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

uint16_t NixPlus::crc16_ccitt(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    uint8_t j = (data[i] ^ (crc >> 8)) & 0xFF;
    crc = CRC16_TABLE[j] ^ (crc << 8);
  }
  return crc;
}

static uint8_t calc_day_of_week(uint16_t y, uint8_t m, uint8_t d) {
  static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (m < 3) y -= 1;
  return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

// NixPlusDisplayLight implementation
light::LightTraits NixPlusDisplayLight::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
  return traits;
}

void NixPlusDisplayLight::write_state(light::LightState *state) {
  if (parent_ != nullptr && parent_->is_handshake_completed() && parent_->is_initial_sync_done()) {
    bool is_on = state->current_values.is_on();
    // In automated mode, light components are marked OFF in UI to indicate clock auto control.
    // Do not forward OFF to the base clock hardware when in automated mode.
    if (parent_->is_automated_mode() && !is_on) {
      return;
    }
    float brightness = state->current_values.get_brightness();
    parent_->set_display_light(is_on, brightness);
  }
}

// NixPlusLight implementation
light::LightTraits NixPlusLight::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supported_color_modes({light::ColorMode::RGB});
  return traits;
}

void NixPlusLight::write_state(light::LightState *state) {
  if (parent_ != nullptr && parent_->is_handshake_completed() && parent_->is_initial_sync_done()) {
    bool is_on = state->current_values.is_on();
    // In automated mode, light components are marked OFF in UI to indicate clock auto control.
    // Do not forward OFF to the base clock hardware when in automated mode.
    if (parent_->is_automated_mode() && !is_on) {
      return;
    }
    if (!is_on) {
      parent_->set_backlight(false, 0, 0, 0, 0);
      return;
    }

    std::string effect = state->get_effect_name().str();
    if (effect == "Colour Cycle - Extremely Slow") {
      parent_->set_backlight_cycling(0);
    } else if (effect == "Colour Cycle - Extra Slow") {
      parent_->set_backlight_cycling(1);
    } else if (effect == "Colour Cycle - Slow") {
      parent_->set_backlight_cycling(2);
    } else if (effect == "Colour Cycle - Medium") {
      parent_->set_backlight_cycling(3);
    } else if (effect == "Colour Cycle - Fast") {
      parent_->set_backlight_cycling(4);
    } else if (effect == "Colour Cycle - Extra Fast") {
      parent_->set_backlight_cycling(5);
    } else {
      float r_f, g_f, b_f;
      state->current_values_as_rgb(&r_f, &g_f, &b_f);

      uint8_t r = static_cast<uint8_t>(std::clamp(roundf(r_f * 31.0f), 0.0f, 31.0f));
      uint8_t g = static_cast<uint8_t>(std::clamp(roundf(g_f * 31.0f), 0.0f, 31.0f));
      uint8_t b = static_cast<uint8_t>(std::clamp(roundf(b_f * 31.0f), 0.0f, 31.0f));

      // Preserve dimmest active step so low brightness does not prematurely black out
      if (r_f > 0.001f && r == 0) r = 1;
      if (g_f > 0.001f && g == 0) g = 1;
      if (b_f > 0.001f && b == 0) b = 1;

      parent_->set_backlight(true, r, g, b, 255);
    }
  }
}

// NixPlus Component implementation
void NixPlus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up NIX+ Universal Network Component...");
  if (user_configured_digits_ > 0) {
    ESP_LOGCONFIG(TAG, "  User-configured digit count: %u", user_configured_digits_);
  } else {
    ESP_LOGCONFIG(TAG, "  Digit count: Auto-detecting via device info command 0x10");
  }
  send_device_info_request();
  read_clock_settings();
  read_backlight_settings();
  request_module_status();
}

void NixPlus::set_display_light_state(light::LightState *s) {
  display_light_state_ = s;
  if (s != nullptr) {
    s->set_default_transition_length(0);
    s->set_gamma_correct(1.0f);
  }
}

void NixPlus::set_backlight_light_state(light::LightState *s) {
  backlight_light_state_ = s;
  if (s != nullptr) {
    s->set_default_transition_length(0);
    s->set_gamma_correct(1.0f);
  }
}

void NixPlus::dump_config() {
  ESP_LOGCONFIG(TAG, "NIX+ Component Configuration:");
  ESP_LOGCONFIG(TAG, "  Connected Model: %s (Type: %u, Digits: %u)", model_name_.c_str(), model_type_, num_digits_);
  ESP_LOGCONFIG(TAG, "  Handshake Status: %s (Serial: %u, HW: %c, FW: v%u.%u)",
                 handshake_completed_ ? "OK" : "PENDING", serial_number_, hw_version_, fw_major_, fw_minor_);
  LOG_SENSOR("  ", "Ambient Light Sensor", ambient_light_sensor_);
  LOG_SENSOR("  ", "Temperature Sensor", temperature_sensor_);
  LOG_BINARY_SENSOR("  ", "Day Mode Sensor", day_mode_sensor_);
  LOG_BINARY_SENSOR("  ", "Night Mode Sensor", night_mode_sensor_);
  LOG_TEXT_SENSOR("  ", "Model Name Sensor", model_name_sensor_);
  LOG_TEXT_SENSOR("  ", "IP Address Sensor", ip_address_sensor_);
  if (display_light_state_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Display Light: Configured");
  }
  if (backlight_light_state_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Backlight Light: Configured");
  }
}

void NixPlus::send_packet(const uint8_t *data, size_t len) {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  if (data != nullptr && len > 0) {
    std::memcpy(frame, data, std::min(len, static_cast<size_t>(62)));
  }

  uint16_t crc = crc16_ccitt(frame, 62);
  frame[62] = (crc >> 8) & 0xFF;
  frame[63] = crc & 0xFF;

  this->write_byte('*');
  this->write_array(frame, 64);
  ESP_LOGD(TAG, "Sent NIX packet opcode 0x%02X (CRC: 0x%04X)", frame[0], crc);
}

void NixPlus::queue_command(const uint8_t *data, size_t len, uint8_t expected_response, uint8_t max_attempts) {
  if (data == nullptr || len == 0) return;

  CommandTransaction tx;
  tx.opcode = data[0];
  std::memset(tx.data, 0, sizeof(tx.data));
  std::memcpy(tx.data, data, std::min(len, static_cast<size_t>(62)));
  tx.expected_response = (expected_response != 0) ? expected_response : tx.opcode;
  tx.max_attempts = max_attempts;
  tx.attempts = 0;
  tx.last_send_ms = 0;
  tx.timeout_ms = 250;

  if (tx_queue_.size() >= 10) {
    ESP_LOGW(TAG, "TX queue full (size %zu), dropping oldest command 0x%02X", tx_queue_.size(), tx_queue_.front().opcode);
    tx_queue_.pop_front();
  }

  tx_queue_.push_back(tx);
  ESP_LOGD(TAG, "Queued command 0x%02X (queue size: %zu)", tx.opcode, tx_queue_.size());

  if (tx_queue_.size() == 1) {
    process_tx_queue();
  }
}

void NixPlus::process_tx_queue() {
  if (tx_queue_.empty()) return;

  uint32_t now = millis();
  auto &tx = tx_queue_.front();

  if (tx.attempts == 0) {
    tx.attempts = 1;
    tx.last_send_ms = now;
    send_packet(tx.data, 64);
  } else if (now - tx.last_send_ms > tx.timeout_ms) {
    if (tx.attempts < tx.max_attempts) {
      tx.attempts++;
      tx.last_send_ms = now;
      send_packet(tx.data, 64);
      ESP_LOGW(TAG, "Retry command 0x%02X (attempt %u/%u, %ums elapsed)",
               tx.opcode, tx.attempts, tx.max_attempts, now - tx.last_send_ms);
    } else {
      ESP_LOGE(TAG, "Command 0x%02X failed: timed out after %u attempts with no clock response",
               tx.opcode, tx.max_attempts);

      // Rollback web UI to confirmed state on failure
      rollback_unconfirmed_state(tx);

      tx_queue_.pop_front();
      if (!tx_queue_.empty()) {
        process_tx_queue();
      }
    }
  }
}

void NixPlus::rollback_unconfirmed_state(const CommandTransaction &tx) {
  if (tx.opcode == 0x08) {
    if (tx.data[0x14] == 3) {
      // Display brightness failed -> roll back to confirmed_display_level_
      if (display_light_state_ != nullptr) {
        float confirmed_br = confirmed_display_level_ / 7.0f;
        if (confirmed_display_level_ == 0) confirmed_br = 0.05f;
        display_light_state_->current_values.set_brightness(confirmed_br);
        display_light_state_->remote_values.set_brightness(confirmed_br);
        display_light_state_->publish_state();
        display_brightness_level_ = confirmed_display_level_;
        last_brightness_level_ = confirmed_display_level_;
        ESP_LOGW(TAG, "Rolled back display brightness to confirmed level %u/7", confirmed_display_level_);
      }
    }
    if (tx.data[5] == 1 || tx.data[5] == 0) {
      // Backlight failed -> roll back to confirmed backlight state
      if (backlight_light_state_ != nullptr) {
        backlight_light_state_->current_values.set_state(confirmed_backlight_power_);
        if (confirmed_backlight_power_) {
          float max_c = static_cast<float>(std::max({confirmed_r_, confirmed_g_, confirmed_b_}));
          float br = (max_c > 0) ? (max_c / 31.0f) : 1.0f;
          float r_n = (max_c > 0) ? (confirmed_r_ / max_c) : 0.0f;
          float g_n = (max_c > 0) ? (confirmed_g_ / max_c) : 0.0f;
          float b_n = (max_c > 0) ? (confirmed_b_ / max_c) : 0.0f;
          backlight_light_state_->current_values.set_brightness(br);
          backlight_light_state_->current_values.set_red(r_n);
          backlight_light_state_->current_values.set_green(g_n);
          backlight_light_state_->current_values.set_blue(b_n);
        }
        backlight_light_state_->remote_values = backlight_light_state_->current_values;
        backlight_light_state_->publish_state();
        backlight_power_state_ = confirmed_backlight_power_;
        last_r_val_ = confirmed_r_;
        last_g_val_ = confirmed_g_;
        last_b_val_ = confirmed_b_;
        ESP_LOGW(TAG, "Rolled back backlight to confirmed state (on=%d, RGB=(%u,%u,%u))",
                 confirmed_backlight_power_, confirmed_r_, confirmed_g_, confirmed_b_);
      }
    }
  } else if (tx.opcode == 0x20) {
    if (tx.data[0x10] == 2 || tx.data[0x10] == 3) {
      if (display_light_state_ != nullptr) {
        display_light_state_->current_values.set_state(confirmed_display_power_);
        display_light_state_->remote_values.set_state(confirmed_display_power_);
        display_light_state_->publish_state();
        display_power_state_ = confirmed_display_power_;
        ESP_LOGW(TAG, "Rolled back display power to confirmed state (%s)", confirmed_display_power_ ? "ON" : "OFF");
      }
    }
  }
}

// Opcode 0x10: Read Device Info (Handshake)
void NixPlus::send_device_info_request() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x10;
  send_packet(frame, 64);
  last_handshake_request_ = millis();
  last_info_poll_ = millis();
}

// Opcode 0x02: Read Clock Settings
void NixPlus::read_clock_settings() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x02;
  send_packet(frame, 64);
  last_settings_request_ = millis();
  ESP_LOGD(TAG, "Sent read clock settings request (0x02)");
}

// Opcode 0x09: Read Backlight Settings
void NixPlus::read_backlight_settings() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x09;
  send_packet(frame, 64);
  last_backlight_settings_request_ = millis();
  ESP_LOGD(TAG, "Sent read backlight settings request (0x09)");
}

// Opcode 0x11: Read Measurement Data (Sensors)
void NixPlus::request_sensors() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x11;
  send_packet(frame, 64);
  last_sensor_request_ = millis();
}

// Time Synchronization
void NixPlus::sync_time_to_clock() {
  manual_time_set_ = false;
  if (time_ != nullptr) {
    ESPTime now = time_->now();
    if (now.year >= 2020 && now.fields_in_range(false, false)) {
      uint8_t wday = (now.day_of_week >= 1 && now.day_of_week <= 7) ?
                     (now.day_of_week - 1) : calc_day_of_week(now.year, now.month, now.day_of_month);

      uint8_t frame[64];
      std::memset(frame, 0, sizeof(frame));
      frame[0] = 0x01; // Set Time and Date
      frame[1] = 0x01; // Subcommand 1: Time & Date only (firmware safeguard)
      frame[2] = now.hour;
      frame[3] = now.minute;
      frame[4] = now.second;
      frame[5] = now.year % 100;
      frame[6] = now.month;
      frame[7] = now.day_of_month;
      frame[8] = wday;
      std::memcpy(&frame[9], clock_settings_, 7);

      queue_command(frame, 64, 0x01, 3);
      ESP_LOGI(TAG, "Pushed internet time sync to clock (0x01): %04d-%02d-%02d %02d:%02d:%02d (wday %u)",
               now.year, now.month, now.day_of_month, now.hour, now.minute, now.second, wday);
      trigger_time_sync();
      return;
    }
  }
  trigger_time_sync();
}

void NixPlus::sync_time_to_clock(ESPTime now) {
  manual_time_ = now;
  manual_time_set_ = true;

  if (now.year < 2000 || !now.fields_in_range(false, false)) {
    ESP_LOGW(TAG, "Invalid time cannot be synced to clock: %04d-%02d-%02d %02d:%02d:%02d",
             now.year, now.month, now.day_of_month, now.hour, now.minute, now.second);
    return;
  }

  uint8_t wday = (now.day_of_week >= 1 && now.day_of_week <= 7) ?
                 (now.day_of_week - 1) : calc_day_of_week(now.year, now.month, now.day_of_month);
  now.day_of_week = wday + 1;

  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x01; // Set Time and Date
  frame[1] = 0x01; // Subcommand 1: Time & Date only (firmware safeguard)
  frame[2] = now.hour;
  frame[3] = now.minute;
  frame[4] = now.second;
  frame[5] = now.year % 100;
  frame[6] = now.month;
  frame[7] = now.day_of_month;
  frame[8] = wday;
  std::memcpy(&frame[9], clock_settings_, 7);

  queue_command(frame, 64, 0x01, 3);
  ESP_LOGI(TAG, "Pushed manual time sync to clock (0x01): %04d-%02d-%02d %02d:%02d:%02d (wday %u)",
           now.year, now.month, now.day_of_month, now.hour, now.minute, now.second, wday);
  trigger_time_sync();
}

void NixPlus::trigger_time_sync() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0xE1;
  frame[4] = 1; // Trigger immediate time sync probe (TIME?) on clock base without touching settings
  queue_command(frame, 64, 0xE1, 3);
  ESP_LOGI(TAG, "Triggered clock time sync probe (0xE1)");
}

// Opcode 0xE0: Read Module Status
void NixPlus::request_module_status() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0xE0;
  send_packet(frame, 64);
  last_module_status_request_ = millis();
  ESP_LOGD(TAG, "Sent read module status request (0xE0)");
}

// Opcode 0xE1: Set Module Function to Active / Enabled
void NixPlus::enable_clock_module() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0xE1;
  frame[2] = 0x01; // Mode: Bit 0 = Always On, Bit 4 = 0 (WiFi enabled)
  frame[3] = 0x01; // ENpin: 1 = Module enabled (pin set low)
  frame[4] = 0x00; // Sync: 0
  frame[5] = 0x01; // Save: 1 = Apply Mode byte to non-volatile flash
  queue_command(frame, 64, 0xE1, 3);
  ESP_LOGI(TAG, "Sent opcode 0xE1 to activate module in clock base (Mode=0x01, ENpin=1, Save=1)");
}

// Display Light Control (Opcode 0x20 power + Opcode 0x08 temporary brightness 0..7)
void NixPlus::set_display_light(bool is_on, float brightness) {
  uint8_t level = static_cast<uint8_t>(std::clamp(roundf(brightness * 7.0f), 0.0f, 7.0f));

  bool was_auto = (ambient_mode_switch_ != nullptr && ambient_mode_switch_->state);
  if (was_auto) {
    ambient_mode_switch_->publish_state(false);
  }

  if (!was_auto && display_initialized_ && is_on == display_power_state_ && level == display_brightness_level_) {
    return;
  }
  bool power_changed = !display_initialized_ || (is_on != display_power_state_);
  display_initialized_ = true;
  display_power_state_ = is_on;
  display_brightness_level_ = level;

  if (!is_on) {
    this->cancel_timeout("wake_finish");
    this->cancel_timeout("backlight_after_wake");

    uint8_t frame[64];
    std::memset(frame, 0, sizeof(frame));
    frame[0] = 0x20;
    frame[0x10] = 2; // PwrCtl: display off / sleep (bit 1=1, bit 0=0)
    queue_command(frame, 64, 0x20, 3);
    ESP_LOGI(TAG, "Queued Display power OFF (0x20)");

    last_brightness_valid_ = false;
    return;
  }

  // Display ON: only send wake packet if power actually changed from OFF to ON
  if (power_changed) {
    uint8_t frame[64];
    std::memset(frame, 0, sizeof(frame));
    frame[0] = 0x20;
    frame[0x10] = 3; // PwrCtl: display on / wake (bit 1=1, bit 0=1)
    queue_command(frame, 64, 0x20, 3);
    ESP_LOGI(TAG, "Queued Display power ON (0x20)");

    last_brightness_valid_ = false;

    // Wait for clock base soft starter to complete before sending brightness
    this->set_timeout("wake_finish", 150, [this, level]() {
      set_display_brightness(level);

      // Re-assert backlight settings if backlight is currently on
      if (backlight_light_state_ != nullptr && backlight_light_state_->remote_values.is_on()) {
        if (active_backlight_effect_ >= 0) {
          set_backlight_cycling(active_backlight_effect_);
        } else if (last_r_val_ > 0 || last_g_val_ > 0 || last_b_val_ > 0) {
          last_rgb_valid_ = false; // ensure set_rgb_color actually transmits to clock
          set_rgb_color(last_r_val_, last_g_val_, last_b_val_, 255);
        }
      }
    });
  } else {
    set_display_brightness(level);
  }
}

void NixPlus::set_display_brightness(uint8_t level) {
  bool was_auto = (ambient_mode_switch_ != nullptr && ambient_mode_switch_->state);
  if (was_auto) {
    ambient_mode_switch_->publish_state(false);
  }

  uint8_t safe_level = std::min(static_cast<uint8_t>(7), level);
  if (!was_auto && last_brightness_valid_ && last_brightness_level_ == safe_level) {
    return;
  }
  last_brightness_level_ = safe_level;
  last_brightness_valid_ = true;

  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x08;
  // Per documentation, setting out-of-range Green (0xFF) safely skips Group A (custom colour),
  // preventing accidental clearing or alteration of backlight settings.
  frame[2] = 0xFF;
  frame[3] = 0xFF;
  frame[4] = 0xFF;
  frame[5] = 0xFF;
  frame[0x13] = safe_level; // Tube brightness level 0..7
  frame[0x14] = 3; // Apply temporary brightness level for both Day and Night (RAM only, no flash save)

  queue_command(frame, 64, 0x08, 3);
  ESP_LOGI(TAG, "Queued temporary display brightness: %u/7 (no flash save)", safe_level);
}

void NixPlus::set_display_power(bool on) {
  if (display_light_state_ != nullptr) {
    if (on) {
      auto call = display_light_state_->turn_on();
      call.set_transition_length(0);
      call.perform();
    } else {
      auto call = display_light_state_->turn_off();
      call.set_transition_length(0);
      call.perform();
    }
  } else {
    set_display_light(on, 1.0f);
  }
}

// Backlight Light Control (Opcode 0x08 custom LED color, no flash save)
void NixPlus::set_backlight(bool is_on, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  bool was_auto = (ambient_mode_switch_ != nullptr && ambient_mode_switch_->state);
  if (was_auto) {
    ambient_mode_switch_->publish_state(false);
  }

  if (!is_on) {
    this->cancel_timeout("backlight_after_wake");
    manual_backlight_active_ = false;
    if (was_auto || backlight_power_state_ || !backlight_initialized_) {
      backlight_power_state_ = false;
      backlight_initialized_ = true;
      active_backlight_effect_ = -1;
      set_rgb_color(0, 0, 0, 0);
    }
    return;
  }

  backlight_power_state_ = true;
  backlight_initialized_ = true;

  set_rgb_color(r, g, b, brightness);
}

void NixPlus::set_rgb_color(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  bool was_auto = (ambient_mode_switch_ != nullptr && ambient_mode_switch_->state);
  if (was_auto) {
    ambient_mode_switch_->publish_state(false);
    last_rgb_valid_ = false;
  }

  uint8_t g_val = std::min(static_cast<uint8_t>(31), g);
  uint8_t r_val = std::min(static_cast<uint8_t>(31), r);
  uint8_t b_val = std::min(static_cast<uint8_t>(31), b);
  if (brightness < 255) {
    float b_scale = brightness / 255.0f;
    g_val = static_cast<uint8_t>(std::clamp(roundf(g_val * b_scale), 0.0f, 31.0f));
    r_val = static_cast<uint8_t>(std::clamp(roundf(r_val * b_scale), 0.0f, 31.0f));
    b_val = static_cast<uint8_t>(std::clamp(roundf(b_val * b_scale), 0.0f, 31.0f));
  }

  if (!was_auto && active_backlight_effect_ == -1 && last_rgb_valid_ && last_g_val_ == g_val && last_r_val_ == r_val && last_b_val_ == b_val) {
    return;
  }
  active_backlight_effect_ = -1;
  last_g_val_ = g_val;
  last_r_val_ = r_val;
  last_b_val_ = b_val;
  last_rgb_valid_ = true;

  if (r_val > 0 || g_val > 0 || b_val > 0) {
    manual_backlight_active_ = true;
  } else {
    manual_backlight_active_ = false;
  }

  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x08;
  frame[2] = g_val; // Green (0-31)
  frame[3] = r_val; // Red (0-31)
  frame[4] = b_val; // Blue (0-31)
  frame[5] = 0x01;  // Save 1: Temporary custom colour in RAM (persists via led_forcedColour without flash wear or timeout)
  frame[0x10] = 0x00; // Disable LED cycle
  frame[0x11] = 0x80; // LED Shift = 0 / Off (MSB=1 applies this). Disables firmware colour offset for static colours.
  frame[0x13] = 0xFF; // Out of range tube brightness to skip Group E
  frame[0x14] = 0;  // Group E Save = 0 (no effect on display brightness)

  queue_command(frame, 64, 0x08, 3);
  ESP_LOGD(TAG, "Queued RGB Backlight set (Save=1 temporary in RAM): R=%u, G=%u, B=%u",
           r_val, g_val, b_val);
}

void NixPlus::set_backlight_cycling(uint8_t mode) {
  manual_backlight_active_ = false;
  if (ambient_mode_switch_ != nullptr && ambient_mode_switch_->state) {
    ambient_mode_switch_->publish_state(false);
  }

  backlight_power_state_ = true;
  backlight_initialized_ = true;

  uint8_t safe_mode = std::min(static_cast<uint8_t>(5), mode);
  if (active_backlight_effect_ == static_cast<int8_t>(safe_mode)) {
    return;
  }
  active_backlight_effect_ = static_cast<int8_t>(safe_mode);
  last_rgb_valid_ = false;

  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x08;
  frame[2] = 0;
  frame[3] = 0;
  frame[4] = 0;
  frame[5] = 0x00; // Clear custom colour so cycling effect takes over
  frame[0x10] = 0x08 | (safe_mode & 0x07); // Enable LED cycle (bit 3 = 1) with mode (0-5)
  // Apply firmware's configured colour offset / shift from cached miscOptions4 (bits 7-6)
  uint8_t fw_shift = (clock_settings_[6] >> 6) & 0x03;
  frame[0x11] = 0x80 | fw_shift;
  frame[0x13] = 0xFF; // Out of range tube brightness to skip Group E
  frame[0x14] = 0;

  queue_command(frame, 64, 0x08, 3);
  ESP_LOGI(TAG, "Queued RGB Backlight cycling effect: mode %u (LEDopt=0x%02X, Shift=0x%02X)",
           safe_mode, frame[0x10], frame[0x11]);
}

void NixPlus::revert_lights() {
  if (!initial_sync_done_) {
    return;
  }
  manual_backlight_active_ = false;
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x08;
  frame[2] = 0;
  frame[3] = 0;
  frame[4] = 0;
  frame[5] = 0x00; // Revert LED colour to programmed value
  frame[0x10] = 0x00; // Clear cycle override
  uint8_t fw_shift = (clock_settings_[6] >> 6) & 0x03;
  frame[0x11] = 0x80 | fw_shift; // Restore firmware shift
  frame[0x13] = 0xFF; // Out of range to skip setting temporary brightness level
  frame[0x14] = 5; // Revert display brightness to programmed values
  queue_command(frame, 64, 0x08, 3);
  last_brightness_level_ = 255;
  last_brightness_valid_ = false;
  last_rgb_valid_ = false;
  active_backlight_effect_ = -1;
  ESP_LOGI(TAG, "Queued revert lights to hardware defaults (Save=5, Save=0, Shift=0x%02X)", frame[0x11]);

  // Turn off the two light components in ESPHome UI to reflect hardware auto control
  if (display_light_state_ != nullptr && display_light_state_->remote_values.is_on()) {
    display_light_state_->current_values.set_state(false);
    display_light_state_->remote_values = display_light_state_->current_values;
    display_light_state_->publish_state();
  }
  if (backlight_light_state_ != nullptr && backlight_light_state_->remote_values.is_on()) {
    backlight_light_state_->current_values.set_state(false);
    backlight_light_state_->remote_values = backlight_light_state_->current_values;
    backlight_light_state_->publish_state();
  }

  if (ambient_mode_switch_ != nullptr && !ambient_mode_switch_->state) {
    ambient_mode_switch_->publish_state(true);
  }

  this->set_timeout("sync_after_revert", 300, [this]() {
    this->request_sensors();
  });
}

void NixPlus::apply_manual_lights() {
  if (!initial_sync_done_) {
    return;
  }
  ESP_LOGI(TAG, "Applying manual lights from ESPHome interface");

  // Restore Display Light component to ON in UI and apply manual brightness
  if (display_light_state_ != nullptr) {
    display_light_state_->current_values.set_state(true);
    display_light_state_->remote_values = display_light_state_->current_values;
    display_light_state_->publish_state();

    float b = display_light_state_->remote_values.get_brightness();
    uint8_t level = static_cast<uint8_t>(std::clamp(roundf(b * 7.0f), 0.0f, 7.0f));
    set_display_brightness(level);
  }

  // Restore Backlight Light component to ON in UI and apply manual color/effect
  if (backlight_light_state_ != nullptr) {
    backlight_light_state_->current_values.set_state(true);
    backlight_light_state_->remote_values = backlight_light_state_->current_values;
    backlight_light_state_->publish_state();
    std::string effect = backlight_light_state_->get_effect_name().str();
    if (effect == "Colour Cycle - Extremely Slow") {
      set_backlight_cycling(0);
    } else if (effect == "Colour Cycle - Extra Slow") {
      set_backlight_cycling(1);
    } else if (effect == "Colour Cycle - Slow") {
      set_backlight_cycling(2);
    } else if (effect == "Colour Cycle - Medium") {
      set_backlight_cycling(3);
    } else if (effect == "Colour Cycle - Fast") {
      set_backlight_cycling(4);
    } else if (effect == "Colour Cycle - Extra Fast") {
      set_backlight_cycling(5);
    } else {
      float r_f, g_f, b_f;
      backlight_light_state_->current_values_as_rgb(&r_f, &g_f, &b_f);
      uint8_t r = static_cast<uint8_t>(std::clamp(roundf(r_f * 31.0f), 0.0f, 31.0f));
      uint8_t g = static_cast<uint8_t>(std::clamp(roundf(g_f * 31.0f), 0.0f, 31.0f));
      uint8_t b = static_cast<uint8_t>(std::clamp(roundf(b_f * 31.0f), 0.0f, 31.0f));
      if (r_f > 0.001f && r == 0) r = 1;
      if (g_f > 0.001f && g == 0) g = 1;
      if (b_f > 0.001f && b == 0) b = 1;
      set_rgb_color(r, g, b, 255);
    }
  }
}


void NixPlus::schedule_state_confirmation() {
  // Confirm device power state and status after 300ms
  this->set_timeout("confirm_clock_info", 300, [this]() {
    this->send_device_info_request();
  });
}

// Opcode 0x20: Value Override & Screen Trigger
void NixPlus::display_number(float value, uint8_t duration_sec) {
  uint8_t hr10 = 255, hr01 = 255, min10 = 255, min01 = 255, sec10 = 255, sec01 = 255;
  uint8_t col = 0;

  uint32_t whole = static_cast<uint32_t>(fabsf(value));
  uint32_t fract = static_cast<uint32_t>(roundf((fabsf(value) - whole) * 100));

  if (num_digits_ == 4) {
    if (fabsf(value) <= 99.99f) {
      if (whole < 10) {
        hr10 = 255; // blanked
      } else {
        hr10 = whole / 10;
      }
      hr01 = whole % 10;
      min10 = fract / 10;
      min01 = fract % 10;
      col = 0x20; // Dot between hr and min
    } else {
      hr10 = (whole / 1000) % 10;
      hr01 = (whole / 100) % 10;
      min10 = (whole / 10) % 10;
      min01 = whole % 10;
    }
  } else { // 6 digits
    if (fabsf(value) <= 9999.99f) {
      hr10 = (whole / 1000) % 10;
      hr01 = (whole / 100) % 10;
      min10 = (whole / 10) % 10;
      min01 = whole % 10;
      sec10 = fract / 10;
      sec01 = fract % 10;
      col = 0x80; // Dot between min and sec
    } else {
      hr10 = (whole / 100000) % 10;
      hr01 = (whole / 10000) % 10;
      min10 = (whole / 1000) % 10;
      min01 = (whole / 100) % 10;
      sec10 = (whole / 10) % 10;
      sec01 = whole % 10;
    }
  }

  uint8_t dur = (duration_sec == 0) ? 255 : (duration_sec == 1 ? 2 : std::min(static_cast<uint8_t>(254), duration_sec));

  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[0x20] = hr10;
  frame[0x21] = hr01;
  frame[0x22] = min10;
  frame[0x23] = min01;
  frame[0x24] = sec10;
  frame[0x25] = sec01;
  frame[0x26] = col;
  frame[0x27] = dur;

  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Display Override: %.2f for %s", value, (dur == 255) ? "indefinite" : (std::to_string(dur) + " sec").c_str());
}

void NixPlus::display_value(const std::string &value_str, uint8_t duration_sec) {
  if (value_str.empty()) {
    clear_display_override();
    return;
  }

  std::string digits;
  int dot_pos = -1;
  for (char c : value_str) {
    if (c >= '0' && c <= '9') {
      digits += c;
    } else if (c == '.' && dot_pos == -1) {
      dot_pos = digits.size();
    }
  }

  if (digits.empty()) {
    ESP_LOGW(TAG, "display_value: no numeric digits found in '%s'", value_str.c_str());
    return;
  }

  uint8_t hr10 = 255, hr01 = 255, min10 = 255, min01 = 255, sec10 = 255, sec01 = 255;
  uint8_t col = 0;
  uint8_t max_d = (num_digits_ == 6) ? 6 : 4;

  if (dot_pos >= 0) {
    std::string before_dot = value_str.substr(0, dot_pos);
    std::string after_dot = value_str.substr(dot_pos + 1);

    std::string clean_before, clean_after;
    for (char c : before_dot) if (c >= '0' && c <= '9') clean_before += c;
    for (char c : after_dot) if (c >= '0' && c <= '9') clean_after += c;

    if (max_d == 4) {
      col = 0x20; // Lower dot between hr and min
      if (clean_before.size() == 1) {
        hr10 = 255;
        hr01 = clean_before[0] - '0';
      } else if (clean_before.size() >= 2) {
        hr10 = clean_before[clean_before.size() - 2] - '0';
        hr01 = clean_before[clean_before.size() - 1] - '0';
      }
      if (clean_after.size() == 1) {
        min10 = clean_after[0] - '0';
        min01 = 0;
      } else if (clean_after.size() >= 2) {
        min10 = clean_after[0] - '0';
        min01 = clean_after[1] - '0';
      }
    } else {
      col = 0x80; // Lower dot between min and sec
      std::vector<uint8_t> b_slots(4, 255);
      size_t b_len = std::min(static_cast<size_t>(4), clean_before.size());
      size_t b_off = 4 - b_len;
      for (size_t i = 0; i < b_len; i++) {
        b_slots[b_off + i] = clean_before[clean_before.size() - b_len + i] - '0';
      }
      hr10 = b_slots[0];
      hr01 = b_slots[1];
      min10 = b_slots[2];
      min01 = b_slots[3];

      if (clean_after.size() == 1) {
        sec10 = clean_after[0] - '0';
        sec01 = 0;
      } else if (clean_after.size() >= 2) {
        sec10 = clean_after[0] - '0';
        sec01 = clean_after[1] - '0';
      }
    }
  } else {
    // Pure integer digits: right-align within available tubes (max_d)
    if (digits.size() > max_d) {
      digits = digits.substr(0, max_d);
    }
    std::vector<uint8_t> slots(max_d, 255);
    size_t offset = max_d - digits.size();
    for (size_t i = 0; i < digits.size(); i++) {
      slots[offset + i] = static_cast<uint8_t>(digits[i] - '0');
    }

    if (max_d == 4) {
      hr10 = slots[0];
      hr01 = slots[1];
      min10 = slots[2];
      min01 = slots[3];
      sec10 = 255;
      sec01 = 255;
    } else {
      hr10 = slots[0];
      hr01 = slots[1];
      min10 = slots[2];
      min01 = slots[3];
      sec10 = slots[4];
      sec01 = slots[5];
    }
  }

  uint8_t dur = (duration_sec == 0) ? 255 : (duration_sec == 1 ? 2 : std::min(static_cast<uint8_t>(254), duration_sec));

  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[0x20] = hr10;
  frame[0x21] = hr01;
  frame[0x22] = min10;
  frame[0x23] = min01;
  frame[0x24] = sec10;
  frame[0x25] = sec01;
  frame[0x26] = col;
  frame[0x27] = dur;

  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Display Override (value '%s', %u digits): [%u, %u, %u, %u, %u, %u] col=0x%02X for %s",
           value_str.c_str(), max_d, hr10, hr01, min10, min01, sec10, sec01, col,
           (dur == 255) ? "indefinite" : (std::to_string(dur) + " sec").c_str());
}

void NixPlus::clear_display_override() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[0x27] = 1; // Duration 1 cancels temporary screen
  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Cleared display override");
}

void NixPlus::show_temperature_screen(uint8_t duration_sec) {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[0x11] = 1; // 1: Show temperature screen for default duration

  // If display was off, wake it up so screen is immediately visible
  if (!display_power_state_) {
    frame[0x10] = 3; // display on / wake
    display_power_state_ = true;
    display_initialized_ = true;
    if (display_light_state_ != nullptr) {
      display_light_state_->current_values.set_state(true);
      display_light_state_->remote_values.set_state(true);
      display_light_state_->publish_state();
    }
  }

  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Show Temperature Screen (default duration)");
}

void NixPlus::show_date_screen(uint8_t duration_sec) {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[0x11] = 2; // 2: Show date screen for default duration

  // If display was off, wake it up so screen is immediately visible
  if (!display_power_state_) {
    frame[0x10] = 3; // display on / wake
    display_power_state_ = true;
    display_initialized_ = true;
    if (display_light_state_ != nullptr) {
      display_light_state_->current_values.set_state(true);
      display_light_state_->remote_values.set_state(true);
      display_light_state_->publish_state();
    }
  }

  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Show Date Screen (default duration)");
}

void NixPlus::show_demo_screen() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[0x11] = 3;
  queue_command(frame, 64, 0x20, 3);
}

// Timer Controls (Opcode 0x20)
void NixPlus::start_timer(uint32_t seconds) {
  if (seconds == 0) {
    stop_timer();
    return;
  }
  if (seconds > 359999) {
    ESP_LOGW(TAG, "Timer duration %u exceeds maximum (359999s = 99h 59m 59s)", seconds);
    seconds = 359999;
  }

  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[1] = (seconds >> 24) & 0xFF;
  frame[2] = (seconds >> 16) & 0xFF;
  frame[3] = (seconds >> 8) & 0xFF;
  frame[4] = seconds & 0xFF;
  frame[5] = 0x0A; // Bit 1: apply up/down (0=down), Bit 3: apply pause (0=unpaused)
  frame[0x27] = 1; // Cancel any active temporary numeric/date screen override

  // If display was off, wake it up so timer is immediately visible
  if (!display_power_state_) {
    frame[0x10] = 3; // display on / wake
    display_power_state_ = true;
    display_initialized_ = true;
    if (display_light_state_ != nullptr) {
      display_light_state_->current_values.set_state(true);
      display_light_state_->remote_values.set_state(true);
      display_light_state_->publish_state();
    }
  }

  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Started timer for %u sec (%02u:%02u:%02u)",
           seconds, seconds / 3600, (seconds % 3600) / 60, seconds % 60);
}

void NixPlus::start_timer(uint8_t hours, uint8_t minutes, uint8_t seconds) {
  uint32_t total = static_cast<uint32_t>(hours) * 3600 + static_cast<uint32_t>(minutes) * 60 + seconds;
  start_timer(total);
}

void NixPlus::stop_timer() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[1] = 0;
  frame[2] = 0;
  frame[3] = 0;
  frame[4] = 0;
  frame[5] = 0x0A; // Down counter apply, unpause apply, 0 seconds cancels timer in PIC32
  frame[0x27] = 1; // Cancel any temporary screen

  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Stopped/cancelled timer");
}

void NixPlus::pause_timer() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[1] = 0xFF; // Value > 359999 so counterTmr is untouched
  frame[5] = 0x0C; // Bit 2 = 1 (pause), Bit 3 = 1 (apply bit 2)

  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Paused timer");
}

void NixPlus::resume_timer() {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x20;
  frame[1] = 0xFF; // Value > 359999 so counterTmr is untouched
  frame[5] = 0x08; // Bit 2 = 0 (unpause), Bit 3 = 1 (apply bit 2)

  queue_command(frame, 64, 0x20, 3);
  ESP_LOGI(TAG, "Resumed timer");
}

// Opcode 0x12: Buzzer Tone
void NixPlus::play_tone(uint16_t frequency_hz, uint16_t duration_ms, uint8_t volume) {
  uint8_t frame[64];
  std::memset(frame, 0, sizeof(frame));
  frame[0] = 0x12;

  uint16_t freq_vol = (frequency_hz & 0x1FFF) | ((volume & 0x07) << 13);
  frame[7] = (freq_vol >> 8) & 0xFF;
  frame[8] = freq_vol & 0xFF;
  frame[9] = static_cast<uint8_t>(std::min(static_cast<uint16_t>(255), static_cast<uint16_t>(duration_ms / 10)));
  frame[10] = 1; // Play tone bit

  queue_command(frame, 64, 0x12, 3);
  ESP_LOGD(TAG, "Queued play tone: %uHz for %ums (volume %u/7)", frequency_hz, duration_ms, volume);
}

void NixPlus::beep(uint8_t count) {
  play_tone(2000, 100 * count, 7);
}

// Binary Frame Processor
void NixPlus::process_binary_frame(const uint8_t *data, size_t len) {
  uint16_t rx_crc = (data[62] << 8) | data[63];
  uint16_t calc_crc = crc16_ccitt(data, 62);

  if (rx_crc != calc_crc) {
    ESP_LOGW(TAG, "Packet CRC mismatch: rx=0x%04X, calc=0x%04X", rx_crc, calc_crc);
    return;
  }

  uint8_t opcode = data[0];

  if (opcode == 0xFF) {
    uint8_t err_cmd = data[1];
    uint8_t err_code = data[2];
    ESP_LOGW(TAG, "Received Packet Error Response (0xFF) from clock: failed cmd=0x%02X, code=0x%02X", err_cmd, err_code);

    if (!tx_queue_.empty() && tx_queue_.front().opcode == err_cmd) {
      auto &tx = tx_queue_.front();
      if (tx.attempts < tx.max_attempts) {
        tx.attempts++;
        tx.last_send_ms = millis();
        send_packet(tx.data, 64);
        ESP_LOGW(TAG, "Immediately retrying command 0x%02X due to 0xFF error (attempt %u/%u)",
                 tx.opcode, tx.attempts, tx.max_attempts);
      } else {
        ESP_LOGE(TAG, "Command 0x%02X failed: exhausted %u attempts after 0xFF error code 0x%02X",
                 tx.opcode, tx.max_attempts, err_code);
        rollback_unconfirmed_state(tx);
        tx_queue_.pop_front();
        if (!tx_queue_.empty()) {
          process_tx_queue();
        }
      }
    }
    return;
  }

  // Check if this incoming packet acknowledges our pending command transaction
  if (!tx_queue_.empty() && (tx_queue_.front().expected_response == opcode || tx_queue_.front().opcode == opcode)) {
    const auto &tx = tx_queue_.front();
    ESP_LOGD(TAG, "Transaction 0x%02X confirmed by clock response 0x%02X", tx.opcode, opcode);

    // Update confirmed hardware states
    if (tx.opcode == 0x08) {
      if (tx.data[0x14] == 3) {
        confirmed_display_level_ = tx.data[0x13];
      }
      if ((tx.data[5] & ~0x80) == 1) {
        confirmed_g_ = tx.data[2];
        confirmed_r_ = tx.data[3];
        confirmed_b_ = tx.data[4];
        confirmed_backlight_power_ = (confirmed_r_ > 0 || confirmed_g_ > 0 || confirmed_b_ > 0);
      } else if (tx.data[5] == 0) {
        confirmed_backlight_power_ = false;
      }
    } else if (tx.opcode == 0x20) {
      if (tx.data[0x10] == 2) {
        confirmed_display_power_ = false;
      } else if (tx.data[0x10] == 3) {
        confirmed_display_power_ = true;
      }
    }

    tx_queue_.pop_front();
    if (!tx_queue_.empty()) {
      process_tx_queue();
    }
  }

  if (opcode == 0x10) {
    model_type_ = data[2];
    serial_number_ = (data[3] << 8) | data[4];
    hw_version_ = data[6];
    uint8_t fw_byte = data[7];
    fw_major_ = ((fw_byte & 0xC0) >> 6) + 1;
    fw_minor_ = fw_byte & 0x3F;

    switch (model_type_) {
      case 1: model_name_ = "DOT6"; break;
      case 2: model_name_ = "NIX4"; break;
      case 3: model_name_ = "NIX6"; break;
      case 4: model_name_ = "VF4";  break;
      case 5: model_name_ = "FL4";  break;
      case 6: model_name_ = "WD1";  break;
      case 7: model_name_ = "NIX4F"; break;
      default: model_name_ = "NIX Clock"; break;
    }

    uint8_t detected_digits = 4;
    if (model_type_ == 1 || model_type_ == 3) {
      detected_digits = 6;
    }

    if (user_configured_digits_ == 0) {
      num_digits_ = detected_digits;
    }

    bool is_first_handshake = !handshake_completed_;
    handshake_completed_ = true;

    if (is_first_handshake) {
      ESP_LOGI(TAG, "Clock Connected: Model=%s (%u digits), Serial=%u, HW=%c, FW=v%u.%u",
               model_name_.c_str(), num_digits_, serial_number_, hw_version_, fw_major_, fw_minor_);

      if (model_name_sensor_ != nullptr) {
        model_name_sensor_->publish_state(model_name_);
      }
      // Request sensors immediately to sync initial brightness and telemetry
      request_sensors();
    }

    // Display Power State confirmation: bit 0 of data[8] is display_offTime
    bool is_disp_off = (data[8] & 0x01) != 0;
    bool disp_power = !is_disp_off;

    display_power_state_ = disp_power;
    display_initialized_ = true;
    confirmed_display_power_ = disp_power;
  } else if (opcode == 0x02) {
    std::memcpy(clock_settings_, &data[9], 7);
    clock_settings_valid_ = true;
    ESP_LOGI(TAG, "Clock settings cached: disp=0x%02X, misc=0x%02X, thresh=%u, night=%u-%u, misc3=0x%02X, misc4=0x%02X",
             data[9], data[10], data[11], data[12], data[13], data[14], data[15]);

    // If internet time is already valid, immediately sync to clock using these verified settings
    if (time_ != nullptr) {
      ESPTime now = time_->now();
      if (now.year >= 2020 && now.fields_in_range(false, false)) {
        sync_time_to_clock();
      }
    }

  } else if (opcode == 0x09) {
    uint8_t custom_g = data[2];
    uint8_t custom_r = data[3];
    uint8_t custom_b = data[4];
    uint8_t time_screen_colour = data[6];
    uint8_t misc_opt2 = data[16];
    bool cycler_enabled = (misc_opt2 & 0x08) != 0;
    uint8_t cycler_mode = misc_opt2 & 0x07;
    int8_t new_cycler_mode = (cycler_enabled && cycler_mode <= 5) ? static_cast<int8_t>(cycler_mode) : -1;

    if (!backlight_settings_valid_) {
      initial_cycler_mode_ = new_cycler_mode;
      cached_time_screen_colour_ = time_screen_colour;
      cached_misc_opt2_ = misc_opt2;
      cached_custom_g_ = custom_g;
      cached_custom_r_ = custom_r;
      cached_custom_b_ = custom_b;
      backlight_settings_valid_ = true;
      ESP_LOGI(TAG, "Clock backlight settings cached (0x09): time_colour=0x%02X, cycler=%s, mode=%u",
               time_screen_colour, cycler_enabled ? "ON" : "OFF", cycler_mode);
    } else {
      bool settings_changed_externally = (time_screen_colour != cached_time_screen_colour_ ||
                                          misc_opt2 != cached_misc_opt2_ ||
                                          custom_g != cached_custom_g_ ||
                                          custom_r != cached_custom_r_ ||
                                          custom_b != cached_custom_b_);
      if (settings_changed_externally) {
        ESP_LOGI(TAG, "Clock backlight settings changed externally: time_colour=0x%02X (was 0x%02X), misc2=0x%02X (was 0x%02X) -> releasing manual backlight override",
                 time_screen_colour, cached_time_screen_colour_, misc_opt2, cached_misc_opt2_);
        cached_time_screen_colour_ = time_screen_colour;
        cached_misc_opt2_ = misc_opt2;
        cached_custom_g_ = custom_g;
        cached_custom_r_ = custom_r;
        cached_custom_b_ = custom_b;

        if (manual_backlight_active_) {
          manual_backlight_active_ = false;
          uint8_t rel_frame[64];
          std::memset(rel_frame, 0, sizeof(rel_frame));
          rel_frame[0] = 0x08;
          rel_frame[5] = 0x00; // Clear forced colour on clock
          rel_frame[0x13] = 0xFF;
          rel_frame[0x14] = 0;
          queue_command(rel_frame, 64, 0x08, 1);
        }

        if (new_cycler_mode != active_backlight_effect_) {
          active_backlight_effect_ = new_cycler_mode;
          bool is_auto = (ambient_mode_switch_ != nullptr && ambient_mode_switch_->state);
          if (backlight_light_state_ != nullptr && !is_auto) {
            if (new_cycler_mode >= 0 && new_cycler_mode <= 5) {
              static const char *const CYCLER_EFFECTS[6] = {
                "Colour Cycle - Extremely Slow",
                "Colour Cycle - Extra Slow",
                "Colour Cycle - Slow",
                "Colour Cycle - Medium",
                "Colour Cycle - Fast",
                "Colour Cycle - Extra Fast"
              };
              auto call = backlight_light_state_->make_call();
              call.set_effect(CYCLER_EFFECTS[new_cycler_mode]);
              call.set_transition_length(0);
              call.perform();
            } else {
              auto call = backlight_light_state_->make_call();
              call.set_effect("None");
              call.set_transition_length(0);
              call.perform();
            }
          }
        }
      }
    }

  } else if (opcode == 0x11) {
    uint16_t raw_als = (data[8] << 8) | data[9];
    int16_t raw_temp = (data[20] << 8) | data[21];
    uint8_t is_day = data[0x29]; // byte 41: ALS_isDay

    // raw_als is in mV (0 to ~2400mV max). Scale to 0-100%
    float als_pct = std::min(100.0f, std::max(0.0f, (raw_als / 2400.0f) * 100.0f));
    float temp_c = raw_temp / 1000.0f;

    if (ambient_light_sensor_ != nullptr) {
      ambient_light_sensor_->publish_state(als_pct);
    }
    if (temperature_sensor_ != nullptr) {
      temperature_sensor_->publish_state(temp_c);
    }
    if (day_mode_sensor_ != nullptr) {
      day_mode_sensor_->publish_state(is_day == 1);
    }
    if (night_mode_sensor_ != nullptr) {
      night_mode_sensor_->publish_state(is_day == 0);
    }

    ESP_LOGD(TAG, "Sensor update: ALS=%.1f%%, Temp=%.1f°C, Day/Night=%s",
             als_pct, temp_c, (is_day == 1) ? "Day" : "Night");

    // Display Brightness Processing
    uint16_t tube_on_time = (data[33] << 8) | data[34];
    const uint32_t *bright_array;
    static const uint32_t BRIGHT_FIL4[8] = {15, 200, 600, 1500, 6000, 8000, 16000, 31500};
    static const uint32_t BRIGHT_VF4[8]  = {500, 1500, 2500, 4000, 8000, 12000, 22000, 31500};
    static const uint32_t BRIGHT_NIX[8]  = {2500, 4000, 7000, 12000, 18000, 22000, 26000, 31500};

    if (model_type_ == 5 || model_type_ == 6) { // FIL4 or WD1
      bright_array = BRIGHT_FIL4;
    } else if (model_type_ == 4) { // VF4
      bright_array = BRIGHT_VF4;
    } else { // NIX4, NIX6, DOT6, NIX4F
      bright_array = BRIGHT_NIX;
    }

    uint8_t clock_level = 0;
    if (tube_on_time <= bright_array[0]) {
      clock_level = 0;
    } else if (tube_on_time >= bright_array[7]) {
      clock_level = 7;
    } else {
      uint32_t min_diff = UINT32_MAX;
      for (uint8_t i = 0; i < 8; i++) {
        uint32_t diff = std::abs(static_cast<int32_t>(tube_on_time) - static_cast<int32_t>(bright_array[i]));
        if (diff < min_diff) {
          min_diff = diff;
          clock_level = i;
        }
      }
    }

    // Backlight Processing
    uint8_t live_g = data[0x30];
    uint8_t live_r = data[0x31];
    uint8_t live_b = data[0x32];

    bool is_automated_mode = (ambient_mode_switch_ != nullptr && ambient_mode_switch_->state);

    if (!initial_sync_done_) {
      // Mirror Display Light on initial startup
      if (display_light_state_ != nullptr) {
        float target_br = clock_level / 7.0f;
        if (clock_level == 0) target_br = 0.05f;

        display_power_state_ = true;
        display_light_state_->current_values.set_state(is_automated_mode ? false : true);
        display_light_state_->current_values.set_brightness(target_br);
        display_light_state_->remote_values = display_light_state_->current_values;
        display_light_state_->publish_state();

        display_brightness_level_ = clock_level;
        confirmed_display_level_ = clock_level;
        last_brightness_level_ = clock_level;
        last_brightness_valid_ = true;
        display_initialized_ = true;
      }

      // Mirror Backlight Light on initial startup
      if (backlight_light_state_ != nullptr) {
        uint8_t max_val = std::max({live_r, live_g, live_b});
        bool want_on = (max_val > 0);

        if (!want_on) {
          manual_backlight_active_ = false;
          backlight_power_state_ = false;
          backlight_initialized_ = true;
          confirmed_backlight_power_ = false;
          last_g_val_ = 0;
          last_r_val_ = 0;
          last_b_val_ = 0;
          last_rgb_valid_ = true;
          active_backlight_effect_ = -1;

          backlight_light_state_->current_values.set_state(false);
          backlight_light_state_->remote_values = backlight_light_state_->current_values;
          backlight_light_state_->publish_state();
        } else {
          float max_scale = 31.0f;
          float live_bl_brightness = std::clamp(static_cast<float>(max_val) / max_scale, 0.05f, 1.0f);
          float r_norm = static_cast<float>(live_r) / max_val;
          float g_norm = static_cast<float>(live_g) / max_val;
          float b_norm = static_cast<float>(live_b) / max_val;

          backlight_power_state_ = true;
          backlight_initialized_ = true;
          confirmed_backlight_power_ = true;
          confirmed_r_ = live_r;
          confirmed_g_ = live_g;
          confirmed_b_ = live_b;
          last_g_val_ = live_g;
          last_r_val_ = live_r;
          last_b_val_ = live_b;
          last_rgb_valid_ = true;

          backlight_light_state_->current_values.set_color_mode(light::ColorMode::RGB);
          backlight_light_state_->current_values.set_red(r_norm);
          backlight_light_state_->current_values.set_green(g_norm);
          backlight_light_state_->current_values.set_blue(b_norm);
          backlight_light_state_->current_values.set_brightness(live_bl_brightness);
          backlight_light_state_->current_values.set_state(is_automated_mode ? false : true);
          backlight_light_state_->remote_values = backlight_light_state_->current_values;
          backlight_light_state_->publish_state();

          if (initial_cycler_mode_ >= 0 && initial_cycler_mode_ <= 5) {
            active_backlight_effect_ = initial_cycler_mode_;
            if (!is_automated_mode) {
              static const char *const CYCLER_EFFECTS[6] = {
                "Colour Cycle - Extremely Slow",
                "Colour Cycle - Extra Slow",
                "Colour Cycle - Slow",
                "Colour Cycle - Medium",
                "Colour Cycle - Fast",
                "Colour Cycle - Extra Fast"
              };
              auto call = backlight_light_state_->make_call();
              call.set_effect(CYCLER_EFFECTS[initial_cycler_mode_]);
              call.set_transition_length(0);
              call.perform();
            }
          } else {
            active_backlight_effect_ = -1;
          }
        }
      }

      initial_sync_done_ = true;
    }

  } else if (opcode == 0x01) {
    ESP_LOGD(TAG, "Clock acknowledged time sync (0x01): status=%u", data[2]);
  } else if (opcode == 0x08) {
    ESP_LOGD(TAG, "Clock acknowledged LED/brightness (0x08): status=0x%02X, saveCode=0x%02X", data[2], data[3]);
  } else if (opcode == 0x20) {
    uint32_t remaining = (static_cast<uint32_t>(data[1]) << 24) |
                         (static_cast<uint32_t>(data[2]) << 16) |
                         (static_cast<uint32_t>(data[3]) << 8) |
                         static_cast<uint32_t>(data[4]);
    uint8_t flags = data[5];
    bool is_paused = (flags & 0x04) != 0;
    bool is_up = (flags & 0x01) != 0;
    ESP_LOGD(TAG, "Clock acknowledged special feature (0x20): counterTmr=%u (paused=%d, up=%d)",
             remaining, is_paused, is_up);
  } else if (opcode == 0xE0) {
    uint8_t state = data[1];
    uint8_t rx_b = data[2];
    uint8_t tx_b = data[3];
    uint8_t mode = data[4];
    uint8_t enpin = data[5];
    uint8_t throt = data[6];
    clock_module_state_ = state;
    clock_module_mode_ = mode;

    ESP_LOGI(TAG, "Clock Module Status (0xE0): state=%u, rx=%u, tx=%u, mode=0x%02X, enpin=%u, throt=%u",
             state, rx_b, tx_b, mode, enpin, throt);

    // If module state is not detected / not active (values < 5), or if WiFi is disabled in Mode bit 4
    if (state < 5 || (mode & 0x10) != 0) {
      if (millis() - last_module_enable_attempt_ > 60000 || last_module_enable_attempt_ == 0) {
        last_module_enable_attempt_ = millis();
        ESP_LOGW(TAG, "Module is inactive in clock base (state=%u, mode=0x%02X). Activating with 0xE1...",
                 state, mode);
        enable_clock_module();
      }
    }
  } else if (opcode == 0xE1) {
    uint8_t err = data[1];
    uint8_t enpin = data[2];
    ESP_LOGI(TAG, "Clock acknowledged module function set (0xE1): err=%u, enpin=%u", err, enpin);
  }
}

std::string NixPlus::get_current_ip_str() {
  if (wifi::global_wifi_component == nullptr || !wifi::global_wifi_component->is_connected()) {
    return "0.0.0.0";
  }
  auto ips = wifi::global_wifi_component->wifi_sta_ip_addresses();
  for (auto &ip : ips) {
    if (ip.is_set() && ip.is_ip4()) {
      char buf[network::IP_ADDRESS_BUFFER_SIZE];
      ip.str_to(buf);
      std::string s = buf;
      if (!s.empty() && s != "0.0.0.0") {
        return s;
      }
    }
  }
  return "0.0.0.0";
}

// ASCII Line Processor (Microcontroller probes)
void NixPlus::process_line(const std::string &line) {
  ESP_LOGD(TAG, "Received UART ASCII line: '%s'", line.c_str());

  if (line.find("STATUS?") != std::string::npos) {
    if (wifi::global_wifi_component != nullptr && wifi::global_wifi_component->is_connected()) {
      this->write_str("ACTIVE\r\n");
      ESP_LOGI(TAG, "Replied ACTIVE to base STATUS? probe");
    } else if (wifi::global_wifi_component != nullptr && wifi::global_wifi_component->has_sta()) {
      this->write_str("CONNECTING\r\n");
      ESP_LOGI(TAG, "Replied CONNECTING to base STATUS? probe");
    } else {
      this->write_str("UNPROVISIONED\r\n");
      ESP_LOGI(TAG, "Replied UNPROVISIONED to base STATUS? probe");
    }

  } else if (line.find("WIFINETIP?") != std::string::npos ||
             line.find("WIFINETP?") != std::string::npos ||
             line.find("WIFINETINFO?") != std::string::npos) {
    std::string ip_str = get_current_ip_str();
    std::string response = "IP: " + ip_str + "\r\n";
    this->write_str(response.c_str());
    ESP_LOGI(TAG, "Replied to IP request (%s) for clock menu display: %s", line.c_str(), response.c_str());

  } else if (line.find("RADIOSUSPEND") != std::string::npos || line.find("RADIORESUME") != std::string::npos) {
    // Acknowledge with OK so clock button menu does not freeze if selected
    this->write_str("OK\r\n");
    ESP_LOGI(TAG, "Replied OK to clock '%s'", line.c_str());

  } else if (line.find("_FWVERSION?") != std::string::npos || line.find("_APPVERSION?") != std::string::npos) {
#ifdef ESPHOME_PROJECT_VERSION
    this->write_str(ESPHOME_PROJECT_VERSION "\r\n");
    ESP_LOGI(TAG, "Replied to firmware version query '%s': %s", line.c_str(), ESPHOME_PROJECT_VERSION);
#else
    this->write_str("0.9.0\r\n");
    ESP_LOGI(TAG, "Replied to firmware version query '%s': 0.9.0", line.c_str());
#endif

  } else if (line.find("TIME?") != std::string::npos) {
    ESPTime now{};
    bool valid = false;
    if (manual_time_set_) {
      now = manual_time_;
      manual_time_set_ = false;
      valid = (now.year >= 2020 && now.fields_in_range(false, false));
    } else if (time_ != nullptr) {
      now = time_->now();
      valid = (now.year >= 2020 && now.fields_in_range(false, false));
    }

    if (valid) {
      char time_buf[48];
      uint8_t wday = (now.day_of_week >= 1 && now.day_of_week <= 7) ?
                     (now.day_of_week - 1) : calc_day_of_week(now.year, now.month, now.day_of_month);
      std::snprintf(time_buf, sizeof(time_buf), "TIME %04d-%02d-%02dT%02d:%02d:%02dZ%u\r\n",
                    now.year, now.month, now.day_of_month, now.hour, now.minute, now.second, wday);
      this->write_str(time_buf);
      ESP_LOGI(TAG, "Replied to clock TIME? sync probe: %s", time_buf);
      return;
    }
    this->write_str("ERR\r\n");
    ESP_LOGW(TAG, "Replied ERR to TIME? (Internet time not synchronized yet)");

  } else if (line.find("RESTART") != std::string::npos) {
    ESP_LOGI(TAG, "Received RESTART command. Rebooting module...");
    this->write_str("OK\r\n");
    this->set_timeout(100, []() { App.safe_reboot(); });

  } else if (line.find("FACTORYRESET") != std::string::npos) {
    ESP_LOGW(TAG, "Received FACTORYRESET command. Resetting preferences and rebooting...");
    this->write_str("OK\r\n");
    this->set_timeout(100, []() {
      global_preferences->reset();
      App.safe_reboot();
    });

  } else if (line.find("SETSNTP:1") != std::string::npos) {
    if (sntp_switch_ != nullptr) {
      sntp_switch_->turn_on();
    }
    this->write_str("OK\r\n");
    ESP_LOGI(TAG, "Enabled SNTP sync via command");

  } else if (line.find("SETSNTP:0") != std::string::npos) {
    if (sntp_switch_ != nullptr) {
      sntp_switch_->turn_off();
    }
    this->write_str("OK\r\n");
    ESP_LOGI(TAG, "Disabled SNTP sync via command");

  } else if (line.find("GETSNTP?") != std::string::npos) {
    bool enabled = (sntp_switch_ != nullptr) ? sntp_switch_->state : true;
    std::string resp = std::string("SNTP:") + (enabled ? "1" : "0") + "\r\n";
    this->write_str(resp.c_str());
    ESP_LOGI(TAG, "Replied to GETSNTP?: %s", resp.c_str());

  } else if (line.find("SETTZ:") != std::string::npos) {
    size_t idx = line.find("SETTZ:");
    std::string tz = line.substr(idx + 6);
    while (!tz.empty() && (tz.back() == '\r' || tz.back() == '\n' || tz.back() == ' ')) {
      tz.pop_back();
    }
    if (!tz.empty()) {
      if (time_zone_text_ != nullptr) {
        auto call = time_zone_text_->make_call();
        call.set_value(tz);
        call.perform();
      }
      this->write_str("OK\r\n");
      ESP_LOGI(TAG, "Set Timezone POSIX rule: %s", tz.c_str());
    } else {
      this->write_str("ERR\r\n");
    }

  } else if (line.find("GETTZ?") != std::string::npos) {
    std::string tz = (time_zone_text_ != nullptr) ? time_zone_text_->state : "";
    std::string resp = "TZ:" + tz + "\r\n";
    this->write_str(resp.c_str());
    ESP_LOGI(TAG, "Replied to GETTZ?: %s", resp.c_str());

  } else if (line.find("AUTOTZ") != std::string::npos) {
    if (detect_timezone_button_ != nullptr) {
      detect_timezone_button_->press();
      this->write_str("OK\r\n");
      ESP_LOGI(TAG, "Triggered Auto-Detect Timezone");
    } else {
      this->write_str("ERR\r\n");
    }

  } else if (line.find("HASCREDS?") != std::string::npos || line.find("HASSTA?") != std::string::npos) {
    bool has_creds = (wifi::global_wifi_component != nullptr && wifi::global_wifi_component->has_sta());
    std::string resp = std::string("CREDS:") + (has_creds ? "1" : "0") + "\r\n";
    this->write_str(resp.c_str());
    ESP_LOGI(TAG, "Replied to HASCREDS?: %s", resp.c_str());
  }
}

void NixPlus::loop() {
  uint32_t now = millis();

  // Reset incomplete binary frame if timeout exceeded
  if (!rx_buffer_.empty() && (now - last_rx_byte_ms_ > 150)) {
    ESP_LOGW(TAG, "Discarding incomplete binary packet of %zu bytes (timeout)", rx_buffer_.size());
    rx_buffer_.clear();
  }

  // Reset incomplete improv frame if timeout exceeded (150ms)
  if (!improv_rx_buffer_.empty() && (now - last_improv_rx_ms_ > 150)) {
    improv_rx_buffer_.clear();
    in_improv_frame_ = false;
    improv_expected_len_ = 0;
  }

  while (this->available()) {
    uint8_t b = this->read();

    // 1. Check if we are currently receiving an active Improv frame
    if (in_improv_frame_) {
      last_improv_rx_ms_ = now;
      improv_rx_buffer_.push_back(b);

      // Byte index 8 is outer data length L (indices 0..5='IMPROV', 6=ver, 7=type, 8=data_len)
      if (improv_rx_buffer_.size() == 9) {
        uint8_t data_len = b;
        // Total frame size: 6 (magic) + 1 (ver) + 1 (type) + 1 (len) + data_len + 1 (checksum)
        improv_expected_len_ = 10 + data_len;
      }

      if (improv_expected_len_ > 0 && improv_rx_buffer_.size() >= improv_expected_len_) {
        // Complete Improv frame received! Forward to improv_serial component
        if (improv_serial::global_improv_serial_component != nullptr) {
          for (uint8_t byte : improv_rx_buffer_) {
            improv_serial::global_improv_serial_component->feed_byte(byte);
          }
        }
        improv_rx_buffer_.clear();
        in_improv_frame_ = false;
        improv_expected_len_ = 0;
      }
      continue;
    }

    // 2. Improv header detection: "IMPROV" (0x49, 0x4D, 0x50, 0x52, 0x4F, 0x56)
    static const uint8_t IMPROV_MAGIC[6] = {'I', 'M', 'P', 'R', 'O', 'V'};
    if (rx_buffer_.empty() && improv_rx_buffer_.size() < 6 && b == IMPROV_MAGIC[improv_rx_buffer_.size()]) {
      improv_rx_buffer_.push_back(b);
      last_improv_rx_ms_ = now;
      if (improv_rx_buffer_.size() == 6) {
        // Matched full "IMPROV" magic header!
        in_improv_frame_ = true;
        improv_expected_len_ = 0;
      }
      continue;
    } else if (!improv_rx_buffer_.empty()) {
      // Sequence broke before full "IMPROV" matched. Flush buffered characters to line_buffer_
      for (uint8_t prev_b : improv_rx_buffer_) {
        if (prev_b >= 32 && prev_b <= 126) {
          line_buffer_ += static_cast<char>(prev_b);
        }
      }
      improv_rx_buffer_.clear();
      // Check if current byte 'b' starts a new "IMPROV" match
      if (rx_buffer_.empty() && b == IMPROV_MAGIC[0]) {
        improv_rx_buffer_.push_back(b);
        last_improv_rx_ms_ = now;
        continue;
      }
    }

    // 3. Binary packet parsing (* prefix + 64 bytes)
    if (rx_buffer_.empty()) {
      if (b == '*') {
        if (!line_buffer_.empty()) {
          process_line(line_buffer_);
          line_buffer_.clear();
        }
        rx_buffer_.push_back(b);
        last_rx_byte_ms_ = now;
        continue;
      }
    } else {
      rx_buffer_.push_back(b);
      last_rx_byte_ms_ = now;
      if (rx_buffer_.size() == 65) {
        process_binary_frame(&rx_buffer_[1], 64);
        rx_buffer_.clear();
      }
      continue;
    }

    // 4. ASCII line parsing for STATUS?, WIFINETIP?, TIME?, etc.
    if (b == '\n' || b == '\r') {
      if (!line_buffer_.empty()) {
        process_line(line_buffer_);
        line_buffer_.clear();
      }
    } else if (b >= 32 && b <= 126) {
      line_buffer_ += static_cast<char>(b);
      last_ascii_rx_ms_ = now;
      // Immediate match for known commands that the PIC32 sends without trailing newline
      if (line_buffer_ == "STATUS?" ||
          line_buffer_ == "WIFINETIP?" ||
          line_buffer_ == "WIFINETP?" ||
          line_buffer_ == "WIFINETINFO?" ||
          line_buffer_ == "RADIOSUSPEND" ||
          line_buffer_ == "RADIORESUME" ||
          line_buffer_ == "_FWVERSION?" ||
          line_buffer_ == "_APPVERSION?" ||
          line_buffer_ == "TIME?" ||
          line_buffer_ == "RESTART" ||
          line_buffer_ == "FACTORYRESET" ||
          line_buffer_ == "GETSNTP?" ||
          line_buffer_ == "SETSNTP:1" ||
          line_buffer_ == "SETSNTP:0" ||
          line_buffer_ == "GETTZ?" ||
          line_buffer_ == "AUTOTZ" ||
          line_buffer_ == "HASCREDS?" ||
          line_buffer_ == "HASSTA?") {
        process_line(line_buffer_);
        line_buffer_.clear();
      } else if (line_buffer_.size() > 64) {
        line_buffer_.clear();
      }
    }
  }

  // Idle timeout check for ASCII commands sent without trailing newline (15ms silence)
  if (!line_buffer_.empty() && (now - last_ascii_rx_ms_ > 15)) {
    process_line(line_buffer_);
    line_buffer_.clear();
  }

  // Periodic IP address publish to text_sensor (updates web UI)
  if (ip_address_sensor_ != nullptr && (now - last_ip_publish_ > 5000 || last_ip_publish_ == 0)) {
    last_ip_publish_ = now;
    std::string current_ip = get_current_ip_str();
    if (current_ip != last_published_ip_) {
      last_published_ip_ = current_ip;
      ip_address_sensor_->publish_state(current_ip);
    }
  }

  // Process any pending transactions in the TX queue (retries on timeout or sends next)
  process_tx_queue();

  // Retry device handshake every 3s until device info (0x10) is received
  if (!handshake_completed_ && (now - last_handshake_request_ > 3000)) {
    send_device_info_request();
  }

  // Retry reading clock settings every 3s until settings (0x02) are received
  if (!clock_settings_valid_ && (now - last_settings_request_ > 3000)) {
    read_clock_settings();
  }

  // Retry reading backlight settings every 3s until settings (0x09) are received
  if (!backlight_settings_valid_ && (now - last_backlight_settings_request_ > 3000)) {
    read_backlight_settings();
  }


  // Periodic polling once handshake is completed:
  // Poll sensors at 15s (4 times per minute) and slower heartbeat for settings/info/status
  if (handshake_completed_ && tx_queue_.empty()) {
    if (now - last_sensor_request_ > 15000) {
      request_sensors();
      last_sensor_request_ = now;
    } else if (now - last_backlight_settings_poll_ > 60000 && (now - last_sensor_request_ > 2000)) {
      read_backlight_settings();
      last_backlight_settings_poll_ = now;
    } else if (now - last_info_poll_ > 60000 && (now - last_sensor_request_ > 4000)) {
      send_device_info_request();
      last_info_poll_ = now;
    } else if (now - last_module_status_request_ > 60000 && (now - last_sensor_request_ > 6000)) {
      request_module_status();
    }
  }
}

}  // namespace nix_plus
}  // namespace esphome
