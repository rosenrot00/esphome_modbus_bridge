// modbus_bridge.h – with configurable poll interval
#pragma once
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>

namespace esphome {
namespace modbus_bridge {

struct TCPClient {
  int fd = -1;
};

struct PendingRequest {
  int client_fd;
  uint8_t header[7];
  std::vector<uint8_t> response;
  bool active = false;
  uint32_t start_time = 0;
  size_t last_size = 0;
  int no_data_counter = 0;
};

class ModbusBridgeComponent : public Component {
 public:
  ModbusBridgeComponent();

  void set_uart_id(uart::UARTComponent *uart) { uart_ = uart; }
  void set_tcp_port(uint16_t port) { tcp_port_ = port; }
  void set_debug(bool debug) { debug_ = debug; }
  void set_tcp_poll_interval(uint32_t interval_ms) { tcp_poll_interval_ms_ = interval_ms; }

  void setup() override;
  void update() override {}

 protected:
  uart::UARTComponent *uart_{};              // UART instance
  int sock_{-1};                             // TCP server socket
  TCPClient clients_[4];                    // Max 4 clients
  PendingRequest pending_request_;          // One request at a time
  uint16_t tcp_port_{502};                  // Default Modbus TCP port
  bool debug_{false};                       // Optional debug output
  uint32_t tcp_poll_interval_ms_{10};       // Configurable polling interval

  void initialize_tcp_server_();            // Setup socket
  void poll_uart_response_();               // Handle UART response
  void append_crc(std::vector<uint8_t> &data); // CRC-16 calculation
  void check_tcp_sockets_();                // Poll TCP clients for data
};

}  // namespace modbus_bridge
}  // namespace esphome
