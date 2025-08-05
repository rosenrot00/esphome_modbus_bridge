#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>
#include <memory>
#include <functional>
#include <initializer_list>
#include <cstring>
#include <deque>

#ifdef USE_ESP8266
#include <ESP8266WiFi.h>
#endif


namespace esphome {
namespace modbus_bridge {

#ifdef USE_ESP8266
struct TCPClient8266 {
  WiFiClient socket;
  uint32_t last_activity = 0;
  int id = -1;  // New field to track client ID/index
};
#endif

#ifdef USE_ESP32
struct TCPClient {
  int fd = -1;
  uint32_t last_activity = 0;
};
#endif

struct PendingRequest {
  int client_fd;  // Used as an identifier for the client on both ESP32 and ESP8266
  uint8_t header[7];
  std::vector<uint8_t> response;
  uint32_t start_time = 0;
  uint32_t last_change = 0;
  std::vector<uint8_t> rtu_data;
};

class ModbusBridgeComponent;  // Forward declaration

class ModbusBridgeComponent : public Component {
 public:
  ModbusBridgeComponent();

  void set_uart_id(uart::UARTComponent *uart) { uart_ = uart; }
  void set_tcp_port(uint16_t port) { tcp_port_ = port; }
  void set_tcp_poll_interval(uint32_t interval_ms) { tcp_poll_interval_ms_ = interval_ms; }
  void set_tcp_client_timeout(uint32_t timeout_ms) { tcp_client_timeout_ms_ = timeout_ms; }
  void set_rtu_response_timeout(uint32_t timeout);
  void set_debug(bool debug);

  void setup() override;

 protected:
  uart::UARTComponent *uart_{nullptr};
  int sock_{-1};
#ifdef USE_ESP32
  TCPClient clients_[4];
#elif defined(USE_ESP8266)
  WiFiServer server_{502};
  std::vector<TCPClient8266> clients_;
#endif
  std::deque<PendingRequest> pending_requests_;
  uint16_t tcp_port_{502};
  bool debug_{false};
  uint32_t tcp_poll_interval_ms_{50};
  uint32_t rtu_poll_interval_ms_{10};
  uint32_t tcp_client_timeout_ms_{60000};
  uint32_t rtu_inactivity_timeout_ms_{20};
  uint32_t rtu_response_timeout_ms_{3000};
  std::vector<uint8_t> temp_buffer_;

  bool polling_active_{false};
  void start_uart_polling_();
  void append_crc(std::vector<uint8_t> &data);
  void initialize_tcp_server_();
  void poll_uart_response_();
  void end_pending_request_();
  void check_tcp_sockets_();
  void handle_tcp_payload(const uint8_t *data, size_t len, int client_fd);
};

}  // namespace modbus_bridge
}  // namespace esphome
