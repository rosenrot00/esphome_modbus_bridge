#pragma once

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <lwip/sockets.h>

namespace esphome {
namespace modbus_bridge {

struct PendingRequest {
  bool active{false};
  int client_fd{-1};
  uint8_t header[4]{0};
  uint32_t start_time{0};
  size_t last_size{0};
  int no_data_counter{0};
  std::vector<uint8_t> response{};
};

class ModbusBridgeComponent : public PollingComponent {
 public:
  ModbusBridgeComponent();

  void setup() override;
  void loop() override;
  void update() override {}

  void set_uart(uart::UARTComponent *uart) { uart_ = uart; }
  void set_tcp_port(uint16_t port) { tcp_port_ = port; }
  void set_debug(bool debug) { debug_ = debug; }

 protected:
  void initialize_tcp_server_();
  void poll_uart_response_();
  void send_response_tcp();
  void log_hex(const char *prefix, const std::vector<uint8_t> &data);
  void append_crc(std::vector<uint8_t> &data);

  uart::UARTComponent *uart_{nullptr};
  int sock_{-1};
  uint16_t tcp_port_{502};
  bool debug_{false};

  PendingRequest pending_request_;

  struct Client {
    int fd{-1};
  };
  Client clients_[4];
};

}  // namespace modbus_bridge
}  // namespace esphome
