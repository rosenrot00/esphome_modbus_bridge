#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/hal.h" // GPIOPin
#include <vector>
#include <functional>
#include <cstring>
#include <deque>
#include "esphome/core/automation.h" // Brings in CallbackManager & Trigger types transitively

#ifdef USE_ESP8266
#include <ESP8266WiFi.h>
#endif

namespace esphome
{
  namespace modbus_bridge
  {

#ifdef USE_ESP8266
    struct TCPClient8266
    {
      WiFiClient socket;
      uint32_t last_activity = 0;
      int id = -1;                      // slot index used as client_id
      bool disconnect_notified = false; // suppress repeated disconnect logs
    };
#endif

#ifdef USE_ESP32
    struct TCPClient
    {
      int fd = -1;
      uint32_t last_activity = 0;
    };
#endif

    struct PendingRequest
    {
      // TCP-side identification
      int client_fd; // client slot index (not a real fd); used to map response back to the TCP client

      // Payload data
      uint8_t header[7];
      std::vector<uint8_t> response;
      std::vector<uint8_t> rtu_data;

      // Timing
      uint32_t start_time = 0;
      size_t last_size = 0; // tracks last observed response size for end-of-frame stability
      uint8_t stable_polls = 0; // consecutive polls with no new UART bytes
    };

    class ModbusBridgeComponent : public Component
    {
    public:
      ModbusBridgeComponent();

      void set_uart_id(uart::UARTComponent *uart) { uart_ = uart; }
      void set_tcp_port(uint16_t port) { tcp_port_ = port; }
      void set_tcp_poll_interval(uint32_t interval_ms) { tcp_poll_interval_ms_ = interval_ms; }
      void set_tcp_client_timeout(uint32_t timeout_ms) { tcp_client_timeout_ms_ = timeout_ms; }
      void set_rtu_response_timeout(uint32_t timeout)
      {
        if (timeout < 10)
          timeout = 10;
        rtu_response_timeout_ms_ = timeout;
      }
      void set_debug(bool debug);
      void set_tcp_allowed_clients(uint8_t allowed)
      {
        if (allowed < 1)
          allowed = 1;
        tcp_allowed_clients_ = allowed;
      }
      void set_crc_bytes_swapped(bool swapped) { crc_bytes_swapped_ = swapped; }
      void set_enabled(bool enabled);
      bool is_enabled() const;

      // Lightweight runtime stats (monotonic counters)
      uint32_t get_frames_in() const;
      uint32_t get_frames_out() const;
      uint32_t get_drops_pid() const;
      uint32_t get_drops_len() const;
      uint32_t get_timeouts() const;
      uint32_t get_clients_connected_total() const;
      uint32_t get_noslot_events() const;
      uint32_t get_preempt_events() const;

      // NEW – optional RS-485 DE/RE control pin (only used if set)
      void set_flow_control_pin(GPIOPin *pin) { flow_control_pin_ = pin; }

      // Bridge-global automation adders (events carry function_code and start_address)
      void add_on_command_sent_callback(std::function<void(int, int)> &&cb) { command_sent_cb_.add(std::move(cb)); }
      void add_on_rtu_receive_callback(std::function<void(int, int)> &&cb) { rtu_receive_cb_.add(std::move(cb)); }
      void add_on_rtu_timeout_callback(std::function<void(int, int)> &&cb) { rtu_timeout_cb_.add(std::move(cb)); }

      void add_on_tcp_started_callback(std::function<void()> &&cb) { tcp_started_cb_.add(std::move(cb)); }
      void add_on_tcp_stopped_callback(std::function<void()> &&cb) { tcp_stopped_cb_.add(std::move(cb)); }
      void add_on_tcp_clients_changed_callback(std::function<void(int)> &&cb) { tcp_clients_changed_cb_.add(std::move(cb)); }

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
      uint8_t tcp_allowed_clients_{2};
      bool crc_bytes_swapped_{false};
      bool enabled_{true};

      bool polling_active_{false};

      // NEW – cached timing for RS-485 toggling
      uint32_t baud_cache_{0};
      uint32_t char_time_us_{0};

      // NEW – optional RS-485 DE/RE pin
      GPIOPin *flow_control_pin_{nullptr};

      // Bridge-global event callbacks
      esphome::CallbackManager<void(int, int)> command_sent_cb_;
      esphome::CallbackManager<void(int, int)> rtu_receive_cb_;
      esphome::CallbackManager<void(int, int)> rtu_timeout_cb_;

      // TCP server state and events
      bool tcp_server_running_{false};
      esphome::CallbackManager<void()> tcp_started_cb_;
      esphome::CallbackManager<void()> tcp_stopped_cb_;
      int tcp_client_count_{0};
      esphome::CallbackManager<void(int)> tcp_clients_changed_cb_;

      // Bridge-wide online/offline state and counters

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

  } // namespace modbus_bridge
} // namespace esphome
