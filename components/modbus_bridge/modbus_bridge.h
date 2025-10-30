#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/hal.h"  // GPIOPin
#include <vector>
#include <memory>
#include <functional>
#include <initializer_list>
#include <cstring>
#include <deque>
#include "esphome/core/callback.h"

#ifdef USE_ESP8266
#include <ESP8266WiFi.h>
#endif

namespace esphome {
namespace modbus_bridge {

#ifdef USE_ESP8266
struct TCPClient8266 {
  WiFiClient socket;
  uint32_t last_activity = 0;
  int id = -1;                 // slot index used as client_id
  bool disconnect_notified = false;  // suppress repeated disconnect logs
};
#endif

#ifdef USE_ESP32
struct TCPClient {
  int fd = -1;
  uint32_t last_activity = 0;
};
#endif

struct PendingRequest {
  // TCP-side identification
  int client_fd;  // Used as an identifier for the client on both ESP32 and ESP8266

  // Payload data
  uint8_t header[7];
  std::vector<uint8_t> response;
  std::vector<uint8_t> rtu_data;

  // Timing
  uint32_t start_time = 0;
  uint32_t last_change = 0;
  size_t last_size = 0;  // tracks last observed response size to reset inactivity timer
};

class ModbusBridgeComponent : public Component {
 public:
  ModbusBridgeComponent();

  void set_uart_id(uart::UARTComponent *uart) { uart_ = uart; }
  void set_tcp_port(uint16_t port) { tcp_port_ = port; }
  void set_tcp_poll_interval(uint32_t interval_ms) { tcp_poll_interval_ms_ = interval_ms; }
  void set_tcp_client_timeout(uint32_t timeout_ms) { tcp_client_timeout_ms_ = timeout_ms; }
  void set_rtu_response_timeout(uint32_t timeout) {
    if (timeout < 10) timeout = 10;
    rtu_response_timeout_ms_ = timeout;
  }
  void set_debug(bool debug);
  void set_tcp_allowed_clients(uint8_t allowed) {
    if (allowed < 1) allowed = 1;
    tcp_allowed_clients_ = allowed;
  }

  // NEW – optional RS-485 DE/RE control pin (only used if set)
  void set_flow_control_pin(GPIOPin *pin) { flow_control_pin_ = pin; }

  // Bridge-global automation adders (events carry function_code and start_address)
  void add_on_command_sent_callback(std::function<void(int, int)> &&cb) { command_sent_cb_.add(std::move(cb)); }
  void add_on_online_callback(std::function<void(int, int)> &&cb)       { online_cb_.add(std::move(cb)); }
  void add_on_offline_callback(std::function<void(int, int)> &&cb)      { offline_cb_.add(std::move(cb)); }
  void add_on_timeout_callback(std::function<void(int, int)> &&cb)      { timeout_cb_.add(std::move(cb)); }

  void setup() override;

 protected:
  uart::UARTComponent *uart_{nullptr};
  int sock_{-1};
#if defined(USE_ESP32)
  std::vector<TCPClient> clients_;
  std::vector<std::vector<uint8_t>> rx_accu_;
#elif defined(USE_ESP8266)
  WiFiServer server_{502};
  std::vector<TCPClient8266> clients_;
  std::vector<std::vector<uint8_t>> rx_accu8266_;
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
  uint8_t tcp_allowed_clients_{4};

  bool polling_active_{false};

  // NEW – cached timing for RS-485 toggling
  uint32_t baud_cache_{0};
  uint32_t char_time_us_{0};

  // NEW – optional RS-485 DE/RE pin
  GPIOPin *flow_control_pin_{nullptr};

  // Bridge-global event callbacks
  esphome::CallbackManager<void(int, int)> command_sent_cb_;
  esphome::CallbackManager<void(int, int)> online_cb_;
  esphome::CallbackManager<void(int, int)> offline_cb_;
  esphome::CallbackManager<void(int, int)> timeout_cb_;

  // Bridge-wide online/offline state and counters
  bool module_offline_{false};
  uint16_t consecutive_timeouts_{0};

  void start_uart_polling_();
  void append_crc(std::vector<uint8_t> &data);
  void initialize_tcp_server_();
  void poll_uart_response_();
  void check_tcp_sockets_();
  void handle_tcp_payload(const uint8_t *data, size_t len, int client_fd);
  void send_to_client_(int slot, const uint8_t *data, size_t len);
  void purge_client_(size_t idx, std::vector<std::vector<uint8_t>> *accu_opt);

  // RS-485 helpers (no-ops when flow_control_pin_ is null)
  void rs485_begin_tx_();
  void rs485_end_tx_();
  void rs485_set_tx_(bool en);
};

}  // namespace modbus_bridge
}  // namespace esphome
