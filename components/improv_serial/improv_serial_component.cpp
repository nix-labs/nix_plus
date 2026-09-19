#include "improv_serial_component.h"
#ifdef USE_WIFI
#include "esphome/core/application.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/version.h"

namespace esphome::improv_serial {

static const char *const TAG = "improv_serial";

ImprovSerialComponent *global_improv_serial_component = nullptr;

void ImprovSerialComponent::setup() {
  global_improv_serial_component = this;

  if (wifi::global_wifi_component->has_sta()) {
    this->state_ = improv::STATE_PROVISIONED;
  } else if (!wifi::global_wifi_component->is_disabled()) {
    wifi::global_wifi_component->start_scanning();
  }
}

void ImprovSerialComponent::loop() {
  const uint32_t now = App.get_loop_component_start_time();
  if (this->last_read_byte_ && (now - this->last_read_byte_ > IMPROV_SERIAL_TIMEOUT)) {
    this->last_read_byte_ = 0;
    this->rx_buffer_.clear();
    ESP_LOGV(TAG, "Timeout");
  }

  while (this->available()) {
    uint8_t byte = this->read();
    if (this->parse_improv_serial_byte_(byte)) {
      this->last_read_byte_ = now;
    } else {
      this->last_read_byte_ = 0;
      this->rx_buffer_.clear();
    }
  }

  if (this->state_ == improv::STATE_PROVISIONING) {
    if (wifi::global_wifi_component->is_connected()) {
      wifi::global_wifi_component->save_wifi_sta(this->connecting_sta_.get_ssid(),
                                                 this->connecting_sta_.get_password());
      this->connecting_sta_ = {};
      this->cancel_timeout("wifi-connect-timeout");
      this->set_state_(improv::STATE_PROVISIONED);

      std::vector<uint8_t> url = this->build_rpc_settings_response_(improv::WIFI_SETTINGS);
      this->send_response_(url);
    }
  }
}

void ImprovSerialComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Improv Serial (UART2):");
  this->check_uart_settings(115200);
}

bool ImprovSerialComponent::feed_byte(uint8_t byte) {
  const uint32_t now = millis();
  if (this->parse_improv_serial_byte_(byte)) {
    this->last_read_byte_ = now;
    return true;
  } else {
    this->last_read_byte_ = 0;
    this->rx_buffer_.clear();
    return false;
  }
}

void ImprovSerialComponent::write_data_(const uint8_t *data, const size_t size) {
  this->tx_header_[TX_LENGTH_IDX] = this->tx_header_[TX_TYPE_IDX] == TYPE_RPC_RESPONSE ? size : 1;

  const bool there_is_data = data != nullptr && size > 0;
  const uint8_t header_checksum_len = there_is_data ? TX_BUFFER_SIZE - 3 : TX_BUFFER_SIZE - 2;
  const uint8_t header_tx_len = there_is_data ? TX_BUFFER_SIZE - 3 : TX_BUFFER_SIZE;

  uint8_t checksum = 0;
  for (uint8_t i = 0; i < header_checksum_len; i++) {
    checksum += this->tx_header_[i];
  }
  if (there_is_data) {
    for (size_t i = 0; i < size; i++) {
      checksum += data[i];
    }
  }
  this->tx_header_[TX_CHECKSUM_IDX] = checksum;

  this->write_array(this->tx_header_, header_tx_len);
  if (there_is_data) {
    this->write_array(data, size);
    this->write_array(&this->tx_header_[TX_CHECKSUM_IDX], 2);
  }
  this->flush();
}

std::vector<uint8_t> ImprovSerialComponent::build_rpc_settings_response_(improv::Command command) {
  std::vector<std::string> urls;
#ifdef USE_IMPROV_SERIAL_NEXT_URL
  {
    char url_buffer[384];
    size_t len = this->get_formatted_next_url_(url_buffer, sizeof(url_buffer));
    if (len > 0) {
      urls.emplace_back(url_buffer, len);
    }
  }
#endif
#ifdef USE_WEBSERVER
  for (auto &ip : wifi::global_wifi_component->wifi_sta_ip_addresses()) {
    if (ip.is_ip4()) {
      char ip_buf[network::IP_ADDRESS_BUFFER_SIZE];
      ip.str_to(ip_buf);
      char webserver_url[7 + network::IP_ADDRESS_BUFFER_SIZE + 1 + 5 + 1];
      snprintf(webserver_url, sizeof(webserver_url), "http://%s:%u", ip_buf, USE_WEBSERVER_PORT);
      urls.emplace_back(webserver_url);
      break;
    }
  }
#endif
  std::vector<uint8_t> data = improv::build_rpc_response(command, urls, false);
  return data;
}

std::vector<uint8_t> ImprovSerialComponent::build_version_info_() {
#ifdef ESPHOME_PROJECT_NAME
  std::vector<std::string> infos = {ESPHOME_PROJECT_NAME, ESPHOME_PROJECT_VERSION, ESPHOME_VARIANT, App.get_name()};
#else
  std::vector<std::string> infos = {"ESPHome", ESPHOME_VERSION, ESPHOME_VARIANT, App.get_name()};
#endif
  std::vector<uint8_t> data = improv::build_rpc_response(improv::GET_DEVICE_INFO, infos, false);
  return data;
}

bool ImprovSerialComponent::parse_improv_serial_byte_(uint8_t byte) {
  size_t at = this->rx_buffer_.size();
  this->rx_buffer_.push_back(byte);
  ESP_LOGV(TAG, "Byte: 0x%02X", byte);
  const uint8_t *raw = &this->rx_buffer_[0];

  return improv::parse_improv_serial_byte(
      at, byte, raw, [this](improv::ImprovCommand command) -> bool { return this->parse_improv_payload_(command); },
      [this](improv::Error error) -> void {
        ESP_LOGW(TAG, "Error decoding payload");
        this->set_error_(error);
      });
}

bool ImprovSerialComponent::parse_improv_payload_(improv::ImprovCommand &command) {
  switch (command.command) {
    case improv::WIFI_SETTINGS: {
      if (wifi::global_wifi_component->is_disabled()) {
        ESP_LOGW(TAG, "Wi-Fi is disabled; cannot provision");
        this->set_error_(improv::ERROR_UNABLE_TO_CONNECT);
        return true;
      }
      wifi::WiFiAP sta{};
      sta.set_ssid(command.ssid.c_str());
      sta.set_password(command.password.c_str());
      this->connecting_sta_ = sta;

      wifi::global_wifi_component->set_sta(sta);
      wifi::global_wifi_component->start_connecting(sta);
      this->set_state_(improv::STATE_PROVISIONING);
      ESP_LOGD(TAG, "Received settings: SSID=%s, password=" LOG_SECRET("%s"), command.ssid.c_str(),
               command.password.c_str());

      this->set_timeout("wifi-connect-timeout", 35000, [this]() { this->on_wifi_connect_timeout_(); });
      return true;
    }
    case improv::GET_CURRENT_STATE:
      if (wifi::global_wifi_component->is_disabled()) {
        this->send_current_state_(improv::STATE_STOPPED);
        return true;
      }
      this->set_state_(this->state_);
      if (this->state_ == improv::STATE_PROVISIONED) {
        std::vector<uint8_t> url = this->build_rpc_settings_response_(improv::GET_CURRENT_STATE);
        this->send_response_(url);
      }
      return true;
    case improv::GET_DEVICE_INFO: {
      std::vector<uint8_t> info = this->build_version_info_();
      this->send_response_(info);
      return true;
    }
    case improv::GET_WIFI_NETWORKS: {
      std::vector<std::string> networks;
      const auto &results = wifi::global_wifi_component->get_scan_result();
      for (auto &scan : results) {
        if (scan.get_is_hidden())
          continue;
        const char *ssid_cstr = scan.get_ssid().c_str();
        bool duplicate = false;
        for (const auto &seen : networks) {
          if (strcmp(seen.c_str(), ssid_cstr) == 0) {
            duplicate = true;
            break;
          }
        }
        if (duplicate)
          continue;
        std::string ssid(ssid_cstr);
        char rssi_buf[5];
        *int8_to_str(rssi_buf, scan.get_rssi()) = '\0';
        std::vector<uint8_t> data =
            improv::build_rpc_response(improv::GET_WIFI_NETWORKS, {ssid, rssi_buf, YESNO(scan.get_with_auth())}, false);
        this->send_response_(data);
        networks.push_back(std::move(ssid));
      }
      std::vector<uint8_t> data =
          improv::build_rpc_response(improv::GET_WIFI_NETWORKS, std::vector<std::string>{}, false);
      this->send_response_(data);
      return true;
    }
    default: {
      ESP_LOGW(TAG, "Unknown payload");
      this->set_error_(improv::ERROR_UNKNOWN_RPC);
      return false;
    }
  }
}

void ImprovSerialComponent::set_state_(improv::State state) {
  this->state_ = state;
  this->send_current_state_(state);
}

void ImprovSerialComponent::send_current_state_(improv::State state) {
  this->tx_header_[TX_TYPE_IDX] = TYPE_CURRENT_STATE;
  this->tx_header_[TX_DATA_IDX] = state;
  this->write_data_();
}

void ImprovSerialComponent::set_error_(improv::Error error) {
  this->tx_header_[TX_TYPE_IDX] = TYPE_ERROR_STATE;
  this->tx_header_[TX_DATA_IDX] = error;
  this->write_data_();
}

void ImprovSerialComponent::send_response_(std::vector<uint8_t> &response) {
  this->tx_header_[TX_TYPE_IDX] = TYPE_RPC_RESPONSE;
  this->write_data_(response.data(), response.size());
}

void ImprovSerialComponent::on_wifi_connect_timeout_() {
  this->set_error_(improv::ERROR_UNABLE_TO_CONNECT);
  this->set_state_(improv::STATE_AUTHORIZED);
  ESP_LOGW(TAG, "Timed out while connecting to Wi-Fi network");
  wifi::global_wifi_component->clear_sta();
}

}  // namespace esphome::improv_serial

#endif
