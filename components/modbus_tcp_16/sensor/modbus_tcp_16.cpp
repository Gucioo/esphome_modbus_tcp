#include "modbus_tcp_16.h"
#include "esphome/core/log.h"
#include "esphome/components/wifi/wifi_component.h"

namespace esphome {
namespace modbus_tcp_16 {

static const char *const TAG = "modbus_tcp_16";

void ModbusTCP16::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Modbus TCP sensor...");
  
  // Initialize TCP client
  tcp_client_ = new AsyncClient();
  
  // Set up callbacks using lambda functions
  tcp_client_->onConnect([this](void* arg, AsyncClient* client) {
    this->on_tcp_connect();
  });
  
  tcp_client_->onDisconnect([this](void* arg, AsyncClient* client) {
    this->on_tcp_disconnect();
  });
  
  tcp_client_->onError([this](void* arg, AsyncClient* client, int8_t error) {
    this->on_tcp_error(error);
  });
  
  tcp_client_->onData([this](void* arg, AsyncClient* client, void* data, size_t len) {
    this->on_tcp_data((const uint8_t*)data, len);
  });
  
  ESP_LOGCONFIG(TAG, "Modbus TCP sensor configured for %s:%d", host_.c_str(), port_);
}

void ModbusTCP16::loop() {
  uint32_t now = millis();
  
  // Check for request timeout
  if (waiting_for_response_ && (now - request_start_time_ > request_timeout_)) {
    ESP_LOGW(TAG, "Modbus request timeout after %d ms", request_timeout_);
    waiting_for_response_ = false;
    disconnect_from_server();
  }
  
  // Check if it's time for next update
  if (!waiting_for_response_ && (now - last_update_ > update_interval_)) {
    if (!connected_) {
      connect_to_server();
    } else {
      send_modbus_request();
    }
    last_update_ = now;
  }
}

void ModbusTCP16::connect_to_server() {
  // Check WiFi connection
  if (!wifi::global_wifi_component->is_connected()) {
    ESP_LOGW(TAG, "WiFi not connected, cannot connect to Modbus server");
    return;
  }
  
  if (tcp_client_->connected()) {
    ESP_LOGD(TAG, "Already connected to server");
    connected_ = true;
    return;
  }
  
  ESP_LOGD(TAG, "Connecting to Modbus TCP server %s:%d", host_.c_str(), port_);
  
  if (!tcp_client_->connect(host_.c_str(), port_)) {
    ESP_LOGE(TAG, "Failed to initiate connection to %s:%d", host_.c_str(), port_);
  }
  // Connection result will be handled in callbacks
}

void ModbusTCP16::disconnect_from_server() {
  if (tcp_client_ && connected_) {
    ESP_LOGD(TAG, "Disconnecting from Modbus server");
    tcp_client_->close(true);
  }
  connected_ = false;
  waiting_for_response_ = false;
}

void ModbusTCP16::cleanup_connection() {
  if (tcp_client_) {
    delete tcp_client_;
    tcp_client_ = nullptr;
  }
  connected_ = false;
  waiting_for_response_ = false;
}

void ModbusTCP16::send_modbus_request() {
  if (!connected_ || waiting_for_response_) {
    ESP_LOGW(TAG, "Cannot send request: connected=%d, waiting=%d", connected_, waiting_for_response_);
    return;
  }
  
  // Increment transaction ID
  transaction_id_++;
  if (transaction_id_ == 0) transaction_id_ = 1; // Avoid 0
  
  // Build Modbus TCP frame
  std::vector<uint8_t> request;
  
  // MBAP Header (Modbus Application Protocol Header)
  request.push_back(transaction_id_ >> 8);      // Transaction ID high byte
  request.push_back(transaction_id_ & 0xFF);    // Transaction ID low byte
  request.push_back(0x00);                      // Protocol ID high byte (always 0)
  request.push_back(0x00);                      // Protocol ID low byte (always 0)
  request.push_back(0x00);                      // Length high byte
  request.push_back(0x06);                      // Length low byte (6 bytes following)
  request.push_back(slave_id_);                 // Unit ID (slave address)
  
  // PDU (Protocol Data Unit) - Read Holding Registers (0x03)
  request.push_back(0x03);                      // Function code
  request.push_back(register_address_ >> 8);    // Starting address high byte
  request.push_back(register_address_ & 0xFF);  // Starting address low byte
  request.push_back(register_count_ >> 8);      // Quantity high byte
  request.push_back(register_count_ & 0xFF);    // Quantity low byte
  
  ESP_LOGD(TAG, "Sending Modbus request: TxID=%d, Slave=%d, Addr=%d, Count=%d", 
           transaction_id_, slave_id_, register_address_, register_count_);
  
  // Send the request
  size_t written = tcp_client_->write((const char*)request.data(), request.size());
  if (written != request.size()) {
    ESP_LOGE(TAG, "Failed to send complete Modbus request: sent %d/%d bytes", written, request.size());
    disconnect_from_server();
    return;
  }
  
  // Mark as waiting for response
  waiting_for_response_ = true;
  request_start_time_ = millis();
  response_buffer_.clear();
  
  ESP_LOGV(TAG, "Modbus request sent successfully");
}

void ModbusTCP16::on_tcp_connect() {
  ESP_LOGD(TAG, "Connected to Modbus TCP server %s:%d", host_.c_str(), port_);
  connected_ = true;
  waiting_for_response_ = false;
  
  // Send initial request
  send_modbus_request();
}

void ModbusTCP16::on_tcp_disconnect() {
  ESP_LOGD(TAG, "Disconnected from Modbus TCP server");
  connected_ = false;
  waiting_for_response_ = false;
  response_buffer_.clear();
}

void ModbusTCP16::on_tcp_error(int8_t error) {
  ESP_LOGE(TAG, "TCP connection error: %d", error);
  connected_ = false;
  waiting_for_response_ = false;
  response_buffer_.clear();
}

void ModbusTCP16::on_tcp_data(const uint8_t *data, size_t len) {
  ESP_LOGV(TAG, "Received %d bytes from Modbus server", len);
  
  // Append received data to buffer
  response_buffer_.insert(response_buffer_.end(), data, data + len);
  
  // Try to process complete response
  process_modbus_response();
}

void ModbusTCP16::process_modbus_response() {
  // Need at least MBAP header + function code + byte count = 9 bytes minimum
  if (response_buffer_.size() < 9) {
    ESP_LOGV(TAG, "Incomplete response, waiting for more data (%d bytes so far)", response_buffer_.size());
    return;
  }
  
  // Parse MBAP header
  uint16_t recv_transaction_id = (response_buffer_[0] << 8) | response_buffer_[1];
  uint16_t protocol_id = (response_buffer_[2] << 8) | response_buffer_[3];
  uint16_t length = (response_buffer_[4] << 8) | response_buffer_[5];
  uint8_t unit_id = response_buffer_[6];
  uint8_t function_code = response_buffer_[7];
  
  ESP_LOGV(TAG, "Response: TxID=%d, Proto=%d, Len=%d, Unit=%d, Func=%02X", 
           recv_transaction_id, protocol_id, length, unit_id, function_code);
  
  // Validate response
  if (recv_transaction_id != transaction_id_) {
    ESP_LOGW(TAG, "Transaction ID mismatch: expected %d, got %d", transaction_id_, recv_transaction_id);
    waiting_for_response_ = false;
    response_buffer_.clear();
    return;
  }
  
  if (protocol_id != 0) {
    ESP_LOGW(TAG, "Invalid protocol ID: %d (should be 0)", protocol_id);
    waiting_for_response_ = false;
    response_buffer_.clear();
    return;
  }
  
  if (unit_id != slave_id_) {
    ESP_LOGW(TAG, "Unit ID mismatch: expected %d, got %d", slave_id_, unit_id);
    waiting_for_response_ = false;
    response_buffer_.clear();
    return;
  }
  
  // Check if we have complete response based on length field
  if (response_buffer_.size() < (6 + length)) {
    ESP_LOGV(TAG, "Waiting for complete response: have %d, need %d bytes", 
             response_buffer_.size(), 6 + length);
    return;
  }
  
  // Process response based on function code
  if (function_code == 0x03) {
    // Read Holding Registers response
    if (response_buffer_.size() < 9) {
      ESP_LOGW(TAG, "Invalid response length for function 0x03");
      waiting_for_response_ = false;
      response_buffer_.clear();
      return;
    }
    
    uint8_t byte_count = response_buffer_[8];
    
    if (response_buffer_.size() < (9 + byte_count)) {
      ESP_LOGV(TAG, "Waiting for register data: have %d, need %d bytes", 
               response_buffer_.size(), 9 + byte_count);
      return;
    }
    
    ESP_LOGD(TAG, "Processing %d bytes of register data", byte_count);
    
    // Extract first register value (16-bit)
    if (byte_count >= 2) {
      uint16_t register_value = (response_buffer_[9] << 8) | response_buffer_[10];
      
      ESP_LOGD(TAG, "Register %d value: %d (0x%04X)", register_address_, register_value, register_value);
      
      // Publish the value as sensor reading
      this->publish_state(register_value);
    } else {
      ESP_LOGW(TAG, "Insufficient register data: %d bytes", byte_count);
    }
    
  } else if (function_code >= 0x80) {
    // Error response (exception)
    if (response_buffer_.size() >= 9) {
      uint8_t exception_code = response_buffer_[8];
      ESP_LOGE(TAG, "Modbus exception: function %02X, exception %02X", 
               function_code & 0x7F, exception_code);
      
      // Publish NaN to indicate error
      this->publish_state(NAN);
    }
  } else {
    ESP_LOGW(TAG, "Unexpected function code in response: %02X", function_code);
  }
  
  // Reset state
  waiting_for_response_ = false;
  response_buffer_.clear();
  
  // Disconnect after successful read (optional - saves resources)
  // Comment out next line if you want to keep connection open
  disconnect_from_server();
}

void ModbusTCP16::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus TCP Sensor:");
  ESP_LOGCONFIG(TAG, "  Host: %s", host_.c_str());
  ESP_LOGCONFIG(TAG, "  Port: %d", port_);
  ESP_LOGCONFIG(TAG, "  Slave ID: %d", slave_id_);
  ESP_LOGCONFIG(TAG, "  Register Address: %d", register_address_);
  ESP_LOGCONFIG(TAG, "  Register Count: %d", register_count_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %d ms", update_interval_);
  LOG_SENSOR("", "Modbus TCP Sensor", this);
}

}  // namespace modbus_tcp_16
}  // namespace esphome
