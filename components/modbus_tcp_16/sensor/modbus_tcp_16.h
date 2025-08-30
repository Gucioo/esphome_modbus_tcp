#pragma once

#include "AsyncTCP.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <vector>
#include <string>

namespace esphome {
namespace modbus_tcp_16 {

static const char *const TAG = "modbus_tcp_16";

class ModbusTCP16 : public Component, public sensor::Sensor {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_WIFI; }

  // Configuration setters
  void set_host(const std::string &host) { host_ = host; }
  void set_port(uint16_t port) { port_ = port; }
  void set_slave_id(uint8_t slave_id) { slave_id_ = slave_id; }
  void set_register_address(uint16_t address) { register_address_ = address; }
  void set_register_count(uint16_t count) { register_count_ = count; }
  void set_update_interval(uint32_t interval) { update_interval_ = interval; }

 protected:
  // Configuration
  std::string host_;
  uint16_t port_{502};
  uint8_t slave_id_{1};
  uint16_t register_address_{0};
  uint16_t register_count_{1};
  uint32_t update_interval_{60000}; // 60 seconds default

  // Connection state
  AsyncClient *tcp_client_{nullptr};
  bool connected_{false};
  uint32_t last_update_{0};
  uint16_t transaction_id_{0};
  std::vector<uint8_t> response_buffer_;
  bool waiting_for_response_{false};
  uint32_t request_timeout_{5000}; // 5 second timeout
  uint32_t request_start_time_{0};

  // Connection management
  void connect_to_server();
  void disconnect_from_server();
  void cleanup_connection();

  // Modbus protocol
  void send_modbus_request();
  void handle_response(const uint8_t *data, size_t len);
  void process_modbus_response();
  uint16_t calculate_crc(const uint8_t *data, size_t len);

  // TCP callbacks
  void on_tcp_connect();
  void on_tcp_disconnect();
  void on_tcp_error(int8_t error);
  void on_tcp_data(const uint8_t *data, size_t len);
};

}  // namespace modbus_tcp_16
}  // namespace esphome
