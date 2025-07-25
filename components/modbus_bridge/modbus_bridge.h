#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>

namespace esphome {
namespace modbus_bridge {

struct TCPClient {
  int fd = -1;
  uint32_t last_activity = 0;  // ⬅️ für Timeout-Überwachung
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
  void set_client_timeout(uint32_t timeout_ms) { client_timeout_ms_ = timeout_ms; }
  void set_response_timeout(uint32_t ms) { modbus_response_timeout_ms_ = ms; }

  void setup() override;

 protected:
  uart::UARTComponent *uart_{nullptr};
  int sock_{-1};
  TCPClient clients_[4];
  PendingRequest pending_request_;
  uint16_t tcp_port_{502};
  bool debug_{false};
  uint32_t tcp_poll_interval_ms_{50};
  uint32_t client_timeout_ms_{30000};
  uint32_t modbus_response_timeout_ms_{1000};

  bool polling_active_{false};
  void start_uart_polling_();
  void append_crc(std::vector<uint8_t> &data);
  void initialize_tcp_server_();
  void poll_uart_response_();
  void check_tcp_sockets_();
};

}  // namespace modbus_bridge
}  // namespace esphome
