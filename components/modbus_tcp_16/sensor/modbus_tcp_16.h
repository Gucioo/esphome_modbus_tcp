#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/network/util.h"
#include "AsyncTCP.h"
#include "esphome/components/wifi/wifi_component.h"
#include "esphome/core/log.h"

class ModbusTCP16 : public Component {
private:
    AsyncClient* tcp_client_;
    
public:
    void setup() override {
        tcp_client_ = new AsyncClient();
        
        // Set up callbacks
        tcp_client_->onConnect([this](void* arg, AsyncClient* client) {
            ESP_LOGD("modbus_tcp", "Connected to server");
        });
        
        tcp_client_->onData([this](void* arg, AsyncClient* client, void* data, size_t len) {
            // Handle received data
            uint8_t* buffer = (uint8_t*)data;
            this->handle_data(buffer, len);
        });
        
        tcp_client_->onError([this](void* arg, AsyncClient* client, int8_t error) {
            ESP_LOGE("modbus_tcp", "Connection error: %d", error);
        });
        
        tcp_client_->onDisconnect([this](void* arg, AsyncClient* client) {
            ESP_LOGD("modbus_tcp", "Disconnected from server");
        });
    }
    
    bool connect(const char* host, uint16_t port) {
        return tcp_client_->connect(host, port);
    }
    
    void send_data(const uint8_t* data, size_t len) {
        if (tcp_client_->connected()) {
            tcp_client_->write((const char*)data, len);
        }
    }
};

namespace esphome {
namespace modbus_tcp_16 {


class ModbusTCP16 :  public sensor::Sensor, public PollingComponent {
 public:
   ModbusTCP16() : PollingComponent(5000) {}

float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

//void update_interval(uint32_t update_interval) { this->update_interval_ = update_interval; }
void set_host(const std::string &host) { this->host_ = host; }
void set_port(uint16_t port) { this->port_ = port; }
void set_functioncode(uint8_t functioncode) { this->functioncode_ = functioncode; }
void set_register_address(uint16_t register_address) { this->register_address_ = register_address; }


void setup() override{
}

void update() override;

// float decode_float(uint8_t *data) {
//    uint32_t raw_value = 0;
//    raw_value = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
//    float value;
//    memcpy(&value, &raw_value, sizeof(float));
//    return value;
//  }



void dump_config() override;

protected:
  uint16_t register_address_;
  uint8_t functioncode_;
//  uint32_t update_interval_;
  uint16_t port_;
  std::string host_;

};

}  // namespace modbus_tcp_16
}  // namespace esphome
