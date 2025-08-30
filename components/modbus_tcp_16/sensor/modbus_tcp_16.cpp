#include "modbus_tcp_16.h"
#include "esphome/core/log.h"
#include "esphome/components/wifi/wifi_component.h"

namespace esphome {
namespace modbus_tcp_16 {

void ModbusTCP16::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Modbus TCP sensor...");
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
    }
    if (connected_) {
      send_modbus_request();
    }
    last_update_ = now;
  }
  
  // If waiting for response, try to read data
  if (waiting_for_response_ && connected_) {
    read_response();
  }
}

void ModbusTCP16::connect_to_server() {
  // Check WiFi connection
  if (!wifi::global_wifi_component->is_connected()) {
    ESP_LOGW(TAG, "WiFi not connected, cannot connect to Modbus server");
    return;
  }
  
  ESP_LOGD(TAG, "Connecting to Modbus TCP server %s:%d", host_.c_str(), port_);
  
  // Create socket
  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %s", strerror(errno));
    return;
  }
  
  // Set socket to non-blocking
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    ESP_LOGE(TAG, "Failed to set socket non-blocking: %s", strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
    return;
  }
  
  // Resolve hostname
  struct hostent *he = gethostbyname(host_.c_str());
  if (he == nullptr) {
    ESP_LOGE(TAG, "Failed to resolve hostname %s", host_.c_str());
    close(socket_fd_);
    socket_fd_ = -1;
    return;
  }
  
  // Setup server address
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port_);
  memcpy(&server_addr.sin_addr.s_addr, he->h_addr, he->h_length);
  
  // Attempt connection
  int result = connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr));
  if (result == 0 || errno == EINPROGRESS) {
    connected_ = true;
    ESP_LOGD(TAG, "Connection initiated to %s:%d", host_.c_str(), port_);
  } else {
    ESP_LOGE(TAG, "Failed to connect to %s:%d: %s", host_.c_str(), port_, strerror(errno));
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

void ModbusTCP16::disconnect_from_server() {
  if (socket_fd_ >= 0) {
    ESP_LOGD(TAG, "Disconnecting from Modbus server");
    close(socket_fd_);
    socket_fd_ = -1;
  }
  connected_ = false;
  waiting_for_response_ = false;
}

void ModbusTCP16::send_modbus_request() {
  if (socket_fd_ < 0 || !connected_ || waiting_for_response_) {
    ESP_LOGW(TAG, "Cannot send request: socket=%d, connected=%d, waiting=%d", 
             socket_fd_, connected_, waiting_for_response_);
    return;
  }
  
  // Check if socket is ready for writing
  fd_set write_fds;
  FD_ZERO(&write_fds);
  FD_SET(socket_fd_, &write_fds);
  
  struct timeval timeout = {0, 0}; // Non-blocking check
  int ready = select(socket_fd_ + 1, nullptr, &write_fds, nullptr, &timeout);
  
  if (ready <= 0) {
    if (ready < 0) {
      ESP_LOGE(TAG, "Socket select error: %s", strerror(errno));
      disconnect_from_server();
    }
    return; // Socket not ready yet
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
  ssize_t written = send(socket_fd_, request.data(), request.size(), MSG_DONTWAIT);
  if (written < 0) {
    if (errno != EAGAIN && errno != EWOULDBLOCK) {
      ESP_LOGE(TAG, "Failed to send Modbus request: %s", strerror(errno));
      disconnect_from_server();
    }
    return;
  } else if (written != (ssize_t)request.size()) {
    ESP_LOGW(TAG, "Partial send: %d/%d bytes", written, request.size());
  }
  
  // Mark as waiting for response
  waiting_for_response_ = true;
  request_start_time_ = millis();
  response_buffer_.clear();
  
  ESP_LOGV(TAG, "Modbus request sent successfully (%d bytes)", written);
}

void ModbusTCP16::read_response() {
  if (socket_fd_ < 0 || !waiting_for_response_) {
    return;
  }
  
  // Check if data is available to read
  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(socket_fd_, &read_fds);
  
  struct timeval timeout = {0, 0}; // Non-blocking check
  int ready = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
  
  if (ready <= 0) {
    return; // No data available or error
  }
  
  uint8_t buffer[256];
  ssize_t received = recv(socket_fd_, buffer, sizeof(buffer), MSG_DONTWAIT);
  
  if (received > 0) {
    ESP_LOGV(TAG, "Received %d bytes from Modbus server", received);
    
    // Append received data to buffer
    response_buffer_.insert(response_buffer_.end(), buffer, buffer + received);
    
    // Try to process complete response
    process_modbus_response();
    
  } else if (received == 0) {
    // Connection closed by remote
    ESP_LOGD(TAG, "Connection closed by server");
    disconnect_from_server();
    
  } else if (errno != EAGAIN && errno != EWOULDBLOCK) {
    // Real error occurred
    ESP_LOGE(TAG, "Socket receive error: %s", strerror(errno));
    disconnect_from_server();
  }
  // EAGAIN/EWOULDBLOCK means no data available yet - that's normal for non-blocking
}

void ModbusTCP16::check_connection() {
  if (socket_fd_ < 0) {
    connected_ = false;
    return;
  }
  
  // Check if connection is still valid
  if (connected_) {
    read_response(); // This will also handle any connection errors
  }
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
    uint8_t byte_count = response_buffer_[8];
    
    if (response_buffer_.size() < (9 + byte_count)) {
      ESP_LOGV(TAG, "Waiting for register data: have %d, need %d bytes", 
               response_buffer_.size(), 9 + byte_count);
      return;
    }
    
    ESP_LOGD(TAG, "Processing %d bytes of register data", byte_count);
    
    // Extract first register value (16-bit, big-endian)
    if (byte_count >= 2) {
      uint16_t register_value = (response_buffer_[9] << 8) | response_buffer_[10];
      
      ESP_LOGD(TAG, "Register %d value: %d (0x%04X)", register_address_, register_value, register_value);
      
      // Publish the value as sensor reading
      this->publish_state(static_cast<float>(register_value));
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
  
  // Disconnect after successful read to save resources
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
