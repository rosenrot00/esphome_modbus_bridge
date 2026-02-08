#include <vector>
#include <functional>
#include <cstring>
#include <cmath>
#include <algorithm>

#ifdef USE_ARDUINO
#include <Arduino.h>
#include "IPAddress.h"
#endif

#ifdef USE_ESP32
#include <lwip/sockets.h>
#include <fcntl.h>
#include <errno.h>
#endif

#include "modbus_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/network/util.h"

namespace esphome
{
  namespace modbus_bridge
  {

    static const char *const TAG = "modbus_bridge";

    // Runtime-configurable (no build flags needed)
    static constexpr size_t kMaxPendingRequests = 32;
    static constexpr size_t kTcpAccuCap = 1024;
    static constexpr size_t kTcpAccuCap8266 = 1024;
    static constexpr size_t kMaxFramesPerLoop = 8;
    // Runtime toggle: preempt oldest same-IP connection when full
    static bool kPreemptSameIP = true;

    // Centralized caps
    static constexpr uint16_t MODBUS_TCP_LEN_CAP = 260;            // UID+PDU (LEN field)
    static constexpr size_t MAX_TCP_READ = 6 + MODBUS_TCP_LEN_CAP; // MBAP(6) + LEN

    // Lightweight runtime counters (file-scope, assume single instance)

    static uint32_t g_frames_in = 0;
    static uint32_t g_frames_out = 0;
    static uint32_t g_drops_pid = 0;
    static uint32_t g_drops_tcp_len = 0;
    static uint32_t g_drops_rtu_incomplete = 0;
    static uint32_t g_timeouts = 0;
    static uint32_t g_clients_connected = 0;
    static uint32_t g_noslot_events = 0;  // number of times a new client was rejected due to no free slot
    static uint32_t g_preempt_events = 0; // number of times we preempted an existing same-IP client (if enabled)

    // --- Helpers to reduce duplication ---------------------------------------------------------

    // Cheap hex conversion for debug logs (truncate to avoid blocking the main loop)
    static inline std::string to_hex(const uint8_t *data, size_t n, size_t max_bytes = 64)
    {
      const size_t m = std::min(n, max_bytes);
      std::string s;
      s.reserve(m * 3 + 24);
      for (size_t i = 0; i < m; ++i)
      {
        char t[4];
        sprintf(t, "%02X ", data[i]);
        s += t;
      }
      if (n > m)
      {
        char tail[32];
        sprintf(tail, "…(+%u bytes)", (unsigned)(n - m));
        s += tail;
      }
      return s;
    }
    static inline std::string to_hex(const std::vector<uint8_t> &v, size_t max_bytes = 64)
    {
      return to_hex(v.data(), v.size(), max_bytes);
    }

    // Drain UART RX (templated to avoid pulling specific UART headers here)
    template <typename U>
    static inline void drain_uart_rx(U *u)
    {
      uint8_t b;
      while (u && u->available())
        u->read_byte(&b);
    }

    // Extracts Modbus PDU function code from an RTU frame (UID + PDU + CRC)
    static inline uint8_t pdu_fc_from_rtu_(const std::vector<uint8_t> &rtu)
    {
      // rtu[0] = UID, rtu[1] = FC
      return rtu.size() >= 2 ? rtu[1] : 0;
    }

    // Extracts start address from common request PDUs (0x01..0x04, 0x05, 0x06, 0x0F, 0x10)

    static inline uint16_t start_addr_from_rtu_(const std::vector<uint8_t> &rtu)
    {
      // For standard requests, start address is at PDU bytes [1..2] → RTU [2], [3]
      if (rtu.size() >= 4)
        return (uint16_t(rtu[2]) << 8) | rtu[3];
      return 0;
    }

    // --- Modbus CRC16 (bitwise, inline) -------------------------------------------------------

    static inline uint16_t modbus_crc(const uint8_t *data, size_t len)
    {
      uint16_t crc = 0xFFFF;
      for (size_t i = 0; i < len; i++)
      {
        crc ^= data[i];
        for (int j = 0; j < 8; j++)
        {
          if (crc & 1)
          {
            crc = (crc >> 1) ^ 0xA001;
          }
          else
          {
            crc >>= 1;
          }
        }
      }
      return crc;
    }

    using FrameHandler = std::function<void(const uint8_t *, size_t, int)>;

    // Build Modbus TCP response from an RTU response
    static inline void build_tcp_from_rtu(const PendingRequest &pending,
                                          const std::vector<uint8_t> &rtu_resp,
                                          std::vector<uint8_t> &out)
    {
      if (rtu_resp.size() < 3)
        return; // invalid RTU frame (uid + fc + crc)
      const size_t current_size = rtu_resp.size();
      const uint16_t pdu_length = static_cast<uint16_t>(current_size - 3); // FC + Data
      const uint16_t mbap_len = static_cast<uint16_t>(pdu_length + 1);     // UID + PDU
      out.clear();
      out.reserve(6 + 2 + 1 + pdu_length);                       // MBAP + LEN + UID + PDU
      out.insert(out.end(), pending.header, pending.header + 4); // TID + PID
      out.push_back((mbap_len >> 8) & 0xFF);
      out.push_back(mbap_len & 0xFF);
      out.push_back(rtu_resp[0]);                                      // UID
      out.insert(out.end(), rtu_resp.begin() + 1, rtu_resp.end() - 2); // PDU (FC+Data), drop CRC
    }

    // Member method to unify sending to a client slot on both platforms
    void ModbusBridgeComponent::send_to_client_(int slot, const uint8_t *data, size_t len)
    {
#if defined(USE_ESP8266)
      if (slot >= 0 && slot < (int)this->clients_.size())
      {
        auto &cl = this->clients_[slot];
        if (cl.socket.connected())
        {
          size_t w = cl.socket.write(data, len);
          if (w != len)
          {
            ESP_LOGW(TAG, "TCP send failed/short client_id=%d wrote=%u/%u", slot, (unsigned)w, (unsigned)len);
            cl.socket.stop();
            this->purge_client_((size_t)slot, &this->rx_accu8266_);
          }
        }
      }
#elif defined(USE_ESP32)
      if (slot >= 0 && slot < (int)this->clients_.size())
      {
        int fd = this->clients_[slot].fd;
        if (fd >= 0)
        {
          int r = send(fd, data, len, 0);
          if (r < 0 || r != (int)len)
          {
            // Treat send errors/short writes as a disconnected client; close and purge slot.
            const unsigned wrote = (r < 0) ? 0U : (unsigned)r;
            ESP_LOGW(TAG, "TCP send failed/short client_id=%d wrote=%u/%u err=%s", slot, wrote, (unsigned)len,
                     (r < 0) ? strerror(errno) : "short");
            this->purge_client_((size_t)slot, &this->rx_accu_);
            close(fd);
            this->clients_[slot].fd = -1;
          }
        }
      }
#endif
    }

#if defined(USE_ESP32)
    static inline void configure_tcp_client_socket_(int fd)
    {
      if (fd < 0)
        return;
      fcntl(fd, F_SETFL, O_NONBLOCK);
      int one = 1;
      setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      int ka = 1;
      setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
#ifdef TCP_KEEPIDLE
      int idle = 30;
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
#endif
#ifdef TCP_KEEPINTVL
      int intvl = 10;
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
#endif
#ifdef TCP_KEEPCNT
      int cnt = 3;
      setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
    }
#endif

    // Extract and process as many complete Modbus-TCP frames as possible from an accumulator
    static inline void process_accu(std::vector<uint8_t> &accu, int client_slot, const FrameHandler &on_frame)
    {
      int processed = 0;
      size_t offset = 0;
      // defensive cap for Modbus-TCP LEN (UID+PDU); typical max ~260
      while (accu.size() >= offset + 7)
      {
        if (processed++ >= (int)kMaxFramesPerLoop)
          break;
        const uint8_t *buf = accu.data() + offset;
        uint16_t len_field = static_cast<uint16_t>((buf[4] << 8) | buf[5]);

        // If LEN is clearly invalid, drop accumulator to recover from poison
        if (len_field < 2 || len_field > MODBUS_TCP_LEN_CAP)
        {
          accu.clear();
          return;
        }

        size_t frame_len = 6UL + static_cast<size_t>(len_field); // MBAP(6) + LEN (UID+PDU)
        if (offset + frame_len > accu.size())
          break; // incomplete; wait for more bytes

        on_frame(buf, frame_len, client_slot);
        offset += frame_len;
      }

      if (offset == 0)
        return;
      if (offset >= accu.size())
      {
        accu.clear();
        return;
      }
      accu.erase(accu.begin(), accu.begin() + offset);
    }

// Tiny counter helper
#ifndef INC
#define INC(x) \
  do           \
  {            \
    ++(x);     \
  } while (0)
#endif
    // Centralized purge for per-client state (accumulator + pending requests + polling flag)
    void ModbusBridgeComponent::purge_client_(size_t idx, std::vector<std::vector<uint8_t>> *accu_opt)
    {
      if (accu_opt && idx < accu_opt->size())
        (*accu_opt)[idx].clear();
      for (auto it = this->pending_requests_.begin(); it != this->pending_requests_.end();)
      {
        if (it->client_fd == (int)idx)
          it = this->pending_requests_.erase(it);
        else
          ++it;
      }
      if (this->pending_requests_.empty())
        this->stop_uart_polling_();
    }

    // --- Optional RS-485 DE/RE support ------------------------------------------

    static inline uint32_t calc_char_time_us_(uint32_t baud)
    {
      // ~11 bits/char (start + 8 data + parity/stop)
      if (baud == 0)
        return 0;
      return static_cast<uint32_t>((11ULL * 1000000ULL) / baud);
    }

    inline void ModbusBridgeComponent::rs485_set_tx_(bool en)
    {
      // Support separate DE and /RE pins.
      // If both are provided, drive them with the same level so a single GPIO can be used for both.
      // Typical RS-485 transceivers use /RE active-low, so tying DE and /RE together works by driving the same signal.
      if (this->de_pin_ != nullptr)
        this->de_pin_->digital_write(en);
      if (this->re_pin_ != nullptr)
        this->re_pin_->digital_write(en);
    }

    inline void ModbusBridgeComponent::rs485_begin_tx_()
    {
      if (this->de_pin_ == nullptr && this->re_pin_ == nullptr)
        return;
      this->rs485_set_tx_(true);
      if (this->char_time_us_ > 0)
      {
        // small pre-delay ~½ char to let the transceiver enable cleanly
        delayMicroseconds(this->char_time_us_ / 2);
      }
    }

    inline void ModbusBridgeComponent::rs485_end_tx_()
    {
      if (this->de_pin_ == nullptr && this->re_pin_ == nullptr)
        return;
      // ensure last stop bit left the wire
      if (this->char_time_us_ > 0)
        delayMicroseconds(this->char_time_us_);
      this->rs485_set_tx_(false);
    }

    // -------------------------------------------------------------------------------------------

    ModbusBridgeComponent::ModbusBridgeComponent()
    {
      this->enabled_ = true;
    }

    void ModbusBridgeComponent::setup()
    {
      this->sock_ = -1;

      // --- Safety guards: UART must be set and baud > 0 ---
      if (this->uart_ == nullptr)
      {
        ESP_LOGE(TAG, "UART not set – aborting setup");
        return;
      }
      const uint32_t _br_setup_guard = this->uart_->get_baud_rate();
      if (_br_setup_guard == 0)
      {
        ESP_LOGE(TAG, "UART baud rate is 0 – aborting setup");
        return;
      }

      // Cache char time; setup optional RS-485 pin
      this->char_time_us_ = calc_char_time_us_(_br_setup_guard);

      // Optional RS-485 DE and /RE pins
      // Drive both with the same level so it works if they are separate GPIOs or the same GPIO is used for both.
      if (this->de_pin_ != nullptr)
      {
        this->de_pin_->setup();              // output
        this->de_pin_->digital_write(false); // RX mode (DE low)
      }
      if (this->re_pin_ != nullptr)
      {
        this->re_pin_->setup();              // output
        this->re_pin_->digital_write(false); // RX mode (/RE low)
      }

      this->set_interval("tcp_server_and_network_check", 1000, [this]()
                         {
      if (!this->enabled_) {
        // Do not start or manage the TCP server while disabled.
        return;
      }

      // Treat "IP available" as "at least one IP address assigned" (backend-agnostic).
      auto ips = network::get_ip_addresses();
      const bool have_ip = !ips.empty();

      if (this->sock_ < 0 && have_ip) {
        ESP_LOGI(TAG, "IP available – initializing TCP server");
        this->initialize_tcp_server_();
      } else if (this->sock_ >= 0 && !have_ip) {
        ESP_LOGW(TAG, "Lost network IP – closing TCP server");
        this->shutdown_tcp_and_pending_();
      }
      });

      this->set_interval("tcp_poll", this->tcp_poll_interval_ms_, [this]()
                         { this->check_tcp_sockets_(); });

      // t3.5 ≈ 3.5 character times, conservatively 11 bits/char
      this->rtu_inactivity_timeout_ms_ = static_cast<uint32_t>(std::ceil((3.5 * 11.0 * 1000.0) / _br_setup_guard) + 1);
      // Use a fixed-size scratch buffer for TCP reads (independent of UART RX size)
      // One full Modbus-TCP frame (MBAP + LEN)
      this->temp_buffer_.resize(MAX_TCP_READ);
      this->rtu_poll_interval_ms_ = this->rtu_inactivity_timeout_ms_ + 2;
      this->stop_uart_polling_();

      // Periodic status log (debug only)
      this->set_interval("status_log", 10000, [this]()
      {
        if (!this->debug_) return;
        size_t clients_active = 0;
        #if defined(USE_ESP8266)
            for (auto &cl : this->clients_) if (cl.socket.connected()) clients_active++;
        #elif defined(USE_ESP32)
            for (auto &cl : this->clients_) if (cl.fd >= 0) clients_active++;
        #endif
        ESP_LOGD(TAG,
                "stats: in=%u out=%u drops(pid)=%u drops(tcp_len)=%u drops(rtu_incomplete)=%u timeouts=%u clients_active=%u clients_total=%u noslot=%u preempt=%u",
                (unsigned)g_frames_in, (unsigned)g_frames_out, (unsigned)g_drops_pid,
                (unsigned)g_drops_tcp_len, (unsigned)g_drops_rtu_incomplete, (unsigned)g_timeouts,
                (unsigned)clients_active, (unsigned)g_clients_connected,
                (unsigned)g_noslot_events, (unsigned)g_preempt_events); });
      }

    void ModbusBridgeComponent::initialize_tcp_server_()
    {
#if defined(USE_ESP8266)
      this->server_ = WiFiServer(this->tcp_port_);
      this->server_.begin();
      this->sock_ = 1; // Dummywert für „Server läuft“
      ESP_LOGI(TAG, "TCP server started on %s:%d", WiFi.localIP().toString().c_str(), this->tcp_port_);
      // TCP event: started (guarded for idempotency)
      if (!this->tcp_server_running_)
      {
        this->tcp_server_running_ = true;
        this->tcp_started_cb_.call();
      }
#elif defined(USE_ESP32)
      this->sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
      if (this->sock_ < 0)
      {
        ESP_LOGE(TAG, "Socket creation failed");
        return;
      }

      fcntl(this->sock_, F_SETFL, O_NONBLOCK);

      // No TCP_NODELAY on server socket itself (set on accept)

      struct sockaddr_in server_addr = {};
      server_addr.sin_family = AF_INET;
      server_addr.sin_port = htons(this->tcp_port_);
      server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

      // Allow quick restart after link flap / reboot
      int reuse = 1;
      setsockopt(this->sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

      if (bind(this->sock_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0)
      {
        ESP_LOGE(TAG, "bind failed: %s", strerror(errno));
        close(this->sock_);
        this->sock_ = -1;
        return;
      }
      int backlog = static_cast<int>(this->tcp_allowed_clients_);
      if (listen(this->sock_, backlog) < 0)
      {
        ESP_LOGE(TAG, "listen failed: %s", strerror(errno));
        close(this->sock_);
        this->sock_ = -1;
        return;
      }

      // Size clients_ vector to allowed clients and reset fds
      this->clients_.assign(this->tcp_allowed_clients_, TCPClient{});
      for (auto &c : this->clients_)
        c.fd = -1;

      auto ips = network::get_ip_addresses();
      if (!ips.empty())
      {
        ESP_LOGI(TAG, "TCP server started on %s:%d", ips[0].str().c_str(), this->tcp_port_);
      }
      else
      {
        ESP_LOGI(TAG, "TCP server started, but no IP address found");
      }
      // TCP event: started (guarded for idempotency)
      if (!this->tcp_server_running_)
      {
        this->tcp_server_running_ = true;
        this->tcp_started_cb_.call();
      }
#endif
    }

    void ModbusBridgeComponent::shutdown_tcp_and_pending_()
    {
#if defined(USE_ESP8266)
      if (this->sock_ >= 0)
        this->server_.stop();
#elif defined(USE_ESP32)
      if (this->sock_ >= 0)
        close(this->sock_);
#endif
      this->sock_ = -1;

#if defined(USE_ESP8266)
      for (size_t i = 0; i < this->clients_.size(); ++i)
      {
        auto &cl = this->clients_[i];
        if (cl.socket.connected())
          cl.socket.stop();
        this->purge_client_(i, &this->rx_accu8266_);
      }
#elif defined(USE_ESP32)
      for (size_t i = 0; i < this->clients_.size(); ++i)
      {
        auto &cl = this->clients_[i];
        this->purge_client_(i, &this->rx_accu_);
        if (cl.fd >= 0)
        {
          close(cl.fd);
          cl.fd = -1;
        }
      }
#endif

      this->pending_requests_.clear();
      this->stop_uart_polling_();
      if (this->uart_ != nullptr)
        drain_uart_rx(this->uart_);

      if (this->tcp_server_running_)
      {
        this->tcp_server_running_ = false;
        this->tcp_stopped_cb_.call();
      }
      if (this->tcp_client_count_ != 0)
      {
        this->tcp_client_count_ = 0;
        this->tcp_clients_changed_cb_.call(0);
      }
    }

    void ModbusBridgeComponent::handle_tcp_payload(const uint8_t *data, size_t len, int client_fd)
    {
      // DoS protection: cap pending requests
      if (this->pending_requests_.size() >= kMaxPendingRequests)
      {
        ESP_LOGW(TAG, "Pending request queue full (%u), dropping frame", (unsigned)this->pending_requests_.size());
        return;
      }
      if (len < 7)
      {
        ESP_LOGW(TAG, "Received too-short frame (%d bytes)", (int)len);
        g_drops_tcp_len++;
        return;
      }

      // MBAP sanity check: Protocol ID must be 0
      if (data[2] != 0 || data[3] != 0)
      {
        ESP_LOGW(TAG, "Non-zero Protocol ID, dropping frame");
        g_drops_pid++;
        return;
      }

      uint16_t modbus_len = (data[4] << 8) | data[5];
      // LEN counts UID + PDU; require at least UID(1)+FC(1)
      if (modbus_len < 2)
      {
        ESP_LOGW(TAG, "Invalid Modbus length (<2), dropping frame: %u", (unsigned)modbus_len);
        g_drops_tcp_len++;
        return;
      }
      // Hard cap to prevent abuse / oversized frames
      if (modbus_len > MODBUS_TCP_LEN_CAP)
      {
        ESP_LOGW(TAG, "Modbus length too large (%u > %u), dropping frame",
                 (unsigned)modbus_len, (unsigned)MODBUS_TCP_LEN_CAP);
        g_drops_tcp_len++;
        return;
      }
      // Function code must be non-zero (first byte of PDU)
      if (len >= 8 && data[7] == 0x00)
      {
        ESP_LOGW(TAG, "Invalid function code 0x00, dropping frame");
        g_drops_pid++;
        return;
      }
      if (modbus_len > this->temp_buffer_.size() - 6)
      {
        ESP_LOGW(TAG, "Invalid Modbus length field: %d (exceeds MAX_TCP_READ)", modbus_len);
        g_drops_tcp_len++;
        return;
      }
      if (len < 6 + modbus_len)
        return;

      uint8_t uid = data[6];

      PendingRequest req;
      req.client_fd = client_fd;
      // Build RTU frame directly in rtu_data: UID + PDU + CRC
      req.rtu_data.clear();
      req.rtu_data.reserve(static_cast<size_t>(modbus_len) + 1 + 2); // UID + PDU + CRC
      req.rtu_data.push_back(uid);
      req.rtu_data.insert(req.rtu_data.end(), data + 7, data + 6 + modbus_len);
      this->append_crc(req.rtu_data);

      if (this->debug_)
      {
        uint32_t now = millis();
        float seconds_ago = -1.0f;
        if (client_fd >= 0 && client_fd < (int)this->clients_.size())
        {
          uint32_t last = this->clients_[client_fd].last_activity;
          seconds_ago = (now - last) / 1000.0f;
        }
        // req.rtu_data[1] is the function code (UID at [0], FC at [1])
        ESP_LOGD(TAG, "TCP->RTU UID: %d, FC: 0x%02X, LEN: %d (client_id=%d, last activity %.3f s ago)",
                 uid,
                 req.rtu_data.size() > 1 ? req.rtu_data[1] : 0,
                 modbus_len, client_fd, seconds_ago);
      }

      memcpy(req.header, data, 7);
      {
        size_t rx_cap = this->uart_->get_rx_buffer_size();
        req.response.reserve(std::max<size_t>(rx_cap, 256));
      }
      req.start_time = 0;
      req.last_size = 0; // ensure deterministic timeout logic
      req.stable_polls = 0;
      g_frames_in++;
      this->pending_requests_.push_back(std::move(req));

      if (this->pending_requests_.size() == 1)
      {
        PendingRequest &cur = this->pending_requests_.front();
        if (this->debug_)
        {
          ESP_LOGD(TAG, "RTU send: %s client_id=%d", to_hex(cur.rtu_data).c_str(), cur.client_fd);
        }
        this->uart_->flush();
        drain_uart_rx(this->uart_);
        this->rs485_begin_tx_();
        this->uart_->write_array(cur.rtu_data);
        this->uart_->flush();
        this->rs485_end_tx_();
        cur.start_time = millis();
        this->start_uart_polling_();
        // Fire command-sent trigger (bridge-global)
        {
          uint8_t fc = pdu_fc_from_rtu_(cur.rtu_data);
          uint16_t addr = start_addr_from_rtu_(cur.rtu_data);
          this->command_sent_cb_.call((int)fc, (int)addr); // Bridge-global event: command sent
        }
      }
    }

    void ModbusBridgeComponent::update_tcp_client_count_()
    {
      int cnt = 0;
#if defined(USE_ESP8266)
      for (auto &cl : this->clients_)
        if (cl.socket.connected())
          ++cnt;
#elif defined(USE_ESP32)
      for (auto &cl : this->clients_)
        if (cl.fd >= 0)
          ++cnt;
#endif
      if (cnt != this->tcp_client_count_)
      {
        this->tcp_client_count_ = cnt;
        this->tcp_clients_changed_cb_.call(cnt);
      }
    }

    void ModbusBridgeComponent::handle_new_client_esp8266_(size_t allowed_clients)
    {
#if defined(USE_ESP8266)
      WiFiClient new_client = this->server_.accept();
      if (!new_client)
        return;

      // Duplicate connection check: same IP and port
      bool duplicate = false;
      for (size_t idx = 0; idx < this->clients_.size(); ++idx)
      {
        auto &ex = this->clients_[idx];
        if (!ex.socket.connected())
          continue;
        if (ex.socket.remoteIP() == new_client.remoteIP() && ex.socket.remotePort() == new_client.remotePort())
        {
          // Reuse the existing slot: drop old socket, keep the new one
          size_t slot_idx = idx;
          ex.socket.stop();
          this->purge_client_(slot_idx, &this->rx_accu8266_);
          ex.socket = new_client;
          ex.socket.setNoDelay(true);
          ex.socket.setTimeout(10);
          ex.last_activity = millis();
          ex.disconnect_notified = false;
          this->update_tcp_client_count_();
          duplicate = true;
          break;
        }
      }
      if (duplicate)
        return;

      // Ensure accumulator size tracks clients_ size and reserve capacity
      if (this->rx_accu8266_.size() < this->clients_.size())
      {
        this->rx_accu8266_.resize(this->clients_.size());
        for (auto &v : this->rx_accu8266_)
          if (v.capacity() < kTcpAccuCap8266)
            v.reserve(kTcpAccuCap8266);
      }

      // Try to reuse a free slot (disconnected client)
      bool placed = false;
      for (size_t idx = 0; idx < this->clients_.size(); ++idx)
      {
        if (!this->clients_[idx].socket.connected())
        {
          this->clients_[idx].socket.stop();
          this->clients_[idx].socket = new_client;
          this->clients_[idx].socket.setNoDelay(true);
          this->clients_[idx].socket.setTimeout(10);
          this->clients_[idx].last_activity = millis();
          this->rx_accu8266_[idx].clear();
          this->clients_[idx].disconnect_notified = false;
          ESP_LOGI(TAG, "TCP connect %s:%u client_id=%d", new_client.remoteIP().toString().c_str(), (unsigned)new_client.remotePort(), (int)idx);
          g_clients_connected++;
          this->update_tcp_client_count_();
          placed = true;
          break;
        }
      }

      // If no free slot, append if under limit
      if (placed)
        return;
      if (this->clients_.size() < allowed_clients)
      {
        TCPClient8266 client;
        client.socket = new_client;
        client.socket.setNoDelay(true);
        client.socket.setTimeout(10);
        client.last_activity = millis();
        this->clients_.push_back(client);
        this->rx_accu8266_.emplace_back();
        if (this->rx_accu8266_.back().capacity() < kTcpAccuCap8266)
          this->rx_accu8266_.back().reserve(kTcpAccuCap8266);
        this->clients_.back().disconnect_notified = false;
        ESP_LOGI(TAG, "TCP connect %s:%u client_id=%d", new_client.remoteIP().toString().c_str(), (unsigned)new_client.remotePort(), (int)(this->clients_.size() - 1));
        g_clients_connected++;
        this->update_tcp_client_count_();
        return;
      }

      bool preempted = false;
      if (kPreemptSameIP)
      {
        // Find oldest active slot with same IP
        size_t victim = SIZE_MAX;
        uint32_t oldest = 0xFFFFFFFFUL;
        for (size_t i = 0; i < allowed_clients && i < this->clients_.size(); ++i)
        {
          auto &cl = this->clients_[i];
          if (!cl.socket.connected())
            continue;
          if (cl.socket.remoteIP() == new_client.remoteIP())
          {
            if (victim == SIZE_MAX || cl.last_activity < oldest)
            {
              victim = i;
              oldest = cl.last_activity;
            }
          }
        }
        if (victim != SIZE_MAX)
        {
          // Close victim and install new client; purge its pending requests
          this->clients_[victim].socket.stop();
          this->purge_client_(victim, &this->rx_accu8266_);
          this->clients_[victim].socket = new_client;
          this->clients_[victim].socket.setNoDelay(true);
          this->clients_[victim].socket.setTimeout(10);
          this->clients_[victim].last_activity = millis();
          this->clients_[victim].disconnect_notified = false;
          INC(g_preempt_events);
          INC(g_clients_connected);
          this->update_tcp_client_count_();
          preempted = true;
        }
      }
      if (!preempted)
      {
        INC(g_noslot_events);
        new_client.stop();
      }
#else
      (void)allowed_clients;
#endif
    }

    void ModbusBridgeComponent::handle_new_client_esp32_(size_t allowed_clients)
    {
#if defined(USE_ESP32)
      struct sockaddr_in client_addr;
      socklen_t addr_len = sizeof(client_addr);
      int newfd = accept(this->sock_, (struct sockaddr *)&client_addr, &addr_len);
      if (newfd < 0)
        return;

      char client_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
      if (this->debug_)
        ESP_LOGD(TAG, "TCP accept %s:%d", client_ip, ntohs(client_addr.sin_port));

      // Check for duplicates: same IP and port
      bool duplicate = false;
      for (auto &c : this->clients_)
      {
        if (c.fd < 0)
          continue;

        struct sockaddr_in existing_addr;
        socklen_t len = sizeof(existing_addr);
        if (getpeername(c.fd, (struct sockaddr *)&existing_addr, &len) == 0)
        {
          if (existing_addr.sin_addr.s_addr == client_addr.sin_addr.s_addr &&
              existing_addr.sin_port == client_addr.sin_port)
          {
            // Reuse the existing slot: replace old fd with new one
            size_t idx = static_cast<size_t>(&c - &this->clients_[0]);
            this->purge_client_(idx, &this->rx_accu_);
            close(c.fd);
            c.fd = newfd;
            c.last_activity = millis();
            this->update_tcp_client_count_();
            duplicate = true;
            break;
          }
        }
      }

      configure_tcp_client_socket_(newfd);
      if (duplicate)
        return;

      bool accepted = false;
      for (size_t idx = 0; idx < allowed_clients; ++idx)
      {
        auto &c = this->clients_[idx];
        if (c.fd < 0)
        {
          c.fd = newfd;
          c.last_activity = millis();
          ESP_LOGI(TAG, "TCP connect %s:%d client_id=%zu", client_ip, ntohs(client_addr.sin_port), idx);
          g_clients_connected++;
          this->update_tcp_client_count_();
          accepted = true;
          break;
        }
      }
      if (accepted)
        return;

      bool preempted = false;
      if (kPreemptSameIP)
      {
        size_t victim = SIZE_MAX;
        uint32_t oldest = 0xFFFFFFFFUL;
        for (size_t i = 0; i < allowed_clients; ++i)
        {
          auto &c = this->clients_[i];
          if (c.fd < 0)
            continue;
          struct sockaddr_in ex;
          socklen_t l = sizeof(ex);
          if (getpeername(c.fd, (struct sockaddr *)&ex, &l) == 0)
          {
            if (ex.sin_addr.s_addr == client_addr.sin_addr.s_addr)
            {
              if (victim == SIZE_MAX || c.last_activity < oldest)
              {
                victim = i;
                oldest = c.last_activity;
              }
            }
          }
        }
        if (victim != SIZE_MAX)
        {
          this->purge_client_(victim, &this->rx_accu_);
          close(this->clients_[victim].fd);
          this->clients_[victim].fd = newfd;
          this->clients_[victim].last_activity = millis();
          INC(g_preempt_events);
          INC(g_clients_connected);
          this->update_tcp_client_count_();
          preempted = true;
        }
      }
      if (!preempted)
      {
        INC(g_noslot_events);
        close(newfd);
      }
#else
      (void)allowed_clients;
#endif
    }

    void ModbusBridgeComponent::check_tcp_sockets_()
    {
#if defined(USE_ESP8266)
      this->check_tcp_sockets_esp8266_();
#elif defined(USE_ESP32)
      this->check_tcp_sockets_esp32_();
#endif
    }

    void ModbusBridgeComponent::check_tcp_sockets_esp8266_()
    {
#if defined(USE_ESP8266)
      // Per-instance RX accumulator
      if (!this->enabled_)
      {
        return; // Do not accept or process any TCP traffic while disabled
      }
      if (this->rx_accu8266_.size() < this->clients_.size())
        this->rx_accu8266_.resize(this->clients_.size());
      if (this->sock_ < 0)
      {
        for (auto &v : this->rx_accu8266_)
          v.clear();
      }
      const size_t allowed_clients = this->tcp_allowed_clients_;
      this->handle_new_client_esp8266_(allowed_clients);

      for (auto it = this->clients_.begin(); it != this->clients_.end();)
      {
        if (!it->socket.connected())
        {
          // keep slot, log only once per transition
          size_t idx = static_cast<size_t>(std::distance(this->clients_.begin(), it));
          if (!it->disconnect_notified)
          {
            ESP_LOGI(TAG, "TCP disconnect client_id=%u", (unsigned)idx);
            it->disconnect_notified = true;
          }
          it->socket.stop();
          this->purge_client_(idx, &this->rx_accu8266_);
          this->update_tcp_client_count_();
          ++it;
          continue;
        }

        if (millis() - it->last_activity > this->tcp_client_timeout_ms_)
        {
          size_t idx = static_cast<size_t>(std::distance(this->clients_.begin(), it));
          ESP_LOGW(TAG, "TCP timeout client_id=%u", (unsigned)idx);
          it->socket.stop();
          it->disconnect_notified = true;
          this->purge_client_(idx, &this->rx_accu8266_);
          this->update_tcp_client_count_();
          ++it;
          continue;
        }

        if (it->socket.available() >= 1)
        {
          int avail = it->socket.available();
          int to_read = std::min(avail, (int)this->temp_buffer_.size());
          int r = it->socket.read(this->temp_buffer_.data(), to_read);
          if (r > 0)
          {
            int client_fd = static_cast<int>(std::distance(this->clients_.begin(), it));
            // Append incoming data
            this->rx_accu8266_[client_fd].insert(this->rx_accu8266_[client_fd].end(), this->temp_buffer_.begin(), this->temp_buffer_.begin() + r);
            process_accu(this->rx_accu8266_[client_fd], client_fd,
                         [&](const uint8_t *buf, size_t flen, int slot)
                         { handle_tcp_payload(buf, flen, slot); });
            // Cap accumulator size to avoid growth on garbage
            const size_t kMaxAccu8266 = kTcpAccuCap8266;
            if (this->rx_accu8266_[client_fd].size() > kMaxAccu8266)
              this->rx_accu8266_[client_fd].clear();
            this->clients_[client_fd].disconnect_notified = false; // receiving traffic confirms connection
            it->last_activity = millis();
          }
        }

        ++it;
      }
#endif
    }

    void ModbusBridgeComponent::check_tcp_sockets_esp32_()
    {
#if defined(USE_ESP32)
      // Per-instance RX accumulator
      if (!this->enabled_)
      {
        return; // Skip accept/read logic while disabled
      }
      this->rx_accu_.resize(this->clients_.size());
      for (auto &v : this->rx_accu_)
      {
        if (v.capacity() < kTcpAccuCap)
          v.reserve(kTcpAccuCap);
      }
      if (this->sock_ < 0)
      {
        for (auto &v : this->rx_accu_)
          v.clear();
        return; // keep accepting/reading even when requests are pending
      }

      const size_t allowed_clients = this->tcp_allowed_clients_;

      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(this->sock_, &read_fds);
      int maxfd = this->sock_;

      uint32_t now = millis();

      for (size_t idx = 0; idx < this->clients_.size(); ++idx)
      {
        auto &c = this->clients_[idx];
        if (idx >= allowed_clients)
        {
          if (c.fd >= 0)
          { // over the configured limit → close
            if (idx < this->rx_accu_.size())
              this->rx_accu_[idx].clear();
            close(c.fd);
            c.fd = -1;
            this->update_tcp_client_count_();
          }
          continue;
        }
        if (c.fd >= 0)
        {
          if (now - c.last_activity > this->tcp_client_timeout_ms_)
          {
            ESP_LOGW(TAG, "TCP timeout client_id=%zu", idx);
            this->purge_client_(idx, &this->rx_accu_);
            close(c.fd);
            c.fd = -1;
            this->update_tcp_client_count_();
            continue;
          }
          FD_SET(c.fd, &read_fds);
          if (c.fd > maxfd)
            maxfd = c.fd;
        }
      }

      // Small timeout to yield to lwIP / logger and avoid starvation when logs are streaming
      struct timeval timeout = {0, 2000}; // 2 ms
      int sel = lwip_select(maxfd + 1, &read_fds, NULL, NULL, &timeout);
      if (sel < 0)
        return;

      if (FD_ISSET(this->sock_, &read_fds))
        this->handle_new_client_esp32_(allowed_clients);

      for (size_t i = 0; i < allowed_clients; ++i)
      {
        auto &c = this->clients_[i];
        if (c.fd >= 0 && FD_ISSET(c.fd, &read_fds))
        {
          int r = recv(c.fd, this->temp_buffer_.data(), this->temp_buffer_.size(), 0);
          if (r == 0)
          {
            ESP_LOGI(TAG, "TCP disconnect client_id=%zu", i);
            this->purge_client_(i, &this->rx_accu_);
            close(c.fd);
            c.fd = -1;
            this->update_tcp_client_count_();
            continue;
          }
          if (r < 0)
          {
            if (errno != EWOULDBLOCK && errno != EAGAIN)
            {
              ESP_LOGW(TAG, "TCP error client_id=%zu err=%s", i, strerror(errno));
              this->purge_client_(i, &this->rx_accu_);
              close(c.fd);
              c.fd = -1;
              this->update_tcp_client_count_();
            }
            continue;
          }
          if (r > 0)
          {
            size_t idx = i;
            // Append incoming bytes to accumulator
            this->rx_accu_[idx].insert(this->rx_accu_[idx].end(), this->temp_buffer_.begin(), this->temp_buffer_.begin() + r);
            process_accu(this->rx_accu_[idx], static_cast<int>(idx),
                         [&](const uint8_t *buf, size_t flen, int slot)
                         { handle_tcp_payload(buf, flen, slot); });
            // Prevent unbounded growth in pathological cases (drop oldest data)
            const size_t kMaxAccu = kTcpAccuCap;
            if (this->rx_accu_[idx].size() > kMaxAccu)
            {
              this->rx_accu_[idx].clear();
            }
            c.last_activity = now;
          }
        }
      }
#endif
    }

    bool ModbusBridgeComponent::is_client_slot_connected_(int slot) const
    {
      if (slot < 0 || slot >= (int)this->clients_.size())
        return false;
#if defined(USE_ESP8266)
      return this->clients_[slot].socket.connected();
#elif defined(USE_ESP32)
      return this->clients_[slot].fd >= 0;
#else
      return false;
#endif
    }

    void ModbusBridgeComponent::send_rtu_request_(PendingRequest &req)
    {
      this->uart_->flush();
      drain_uart_rx(this->uart_);
      this->rs485_begin_tx_();
      this->uart_->write_array(req.rtu_data);
      this->uart_->flush();
      this->rs485_end_tx_();
      req.start_time = millis();

      if (this->debug_)
      {
        ESP_LOGD(TAG, "RTU send: %s client_id=%d", to_hex(req.rtu_data).c_str(), req.client_fd);
      }

      uint8_t fc = pdu_fc_from_rtu_(req.rtu_data);
      uint16_t addr = start_addr_from_rtu_(req.rtu_data);
      this->command_sent_cb_.call((int)fc, (int)addr);
    }

    bool ModbusBridgeComponent::finish_current_and_send_next_()
    {
      if (!this->pending_requests_.empty())
        this->pending_requests_.pop_front();

      while (!this->pending_requests_.empty())
      {
        auto &next = this->pending_requests_.front();
        if (!this->is_client_slot_connected_(next.client_fd))
        {
          this->pending_requests_.pop_front();
          continue;
        }
        this->send_rtu_request_(next);
        return true;
      }

      this->stop_uart_polling_();
      return false;
    }

    void ModbusBridgeComponent::fire_rtu_timeout_for_request_(const PendingRequest &req)
    {
      uint8_t fc = pdu_fc_from_rtu_(req.rtu_data);
      uint16_t addr = start_addr_from_rtu_(req.rtu_data);
      this->rtu_timeout_cb_.call((int)fc, (int)addr);
    }

    void ModbusBridgeComponent::read_uart_response_bytes_(PendingRequest &req)
    {
      size_t avail = this->uart_->available();
      if (!avail)
        return;
      req.response.reserve(req.response.size() + avail);
      for (size_t i = 0; i < avail; ++i)
      {
        uint8_t b;
        if (this->uart_->read_byte(&b))
          req.response.push_back(b);
        else
          break;
      }
    }

    void ModbusBridgeComponent::start_uart_polling_()
    {
      if (this->polling_active_)
        return;
      this->set_interval("modbus_rx_poll", this->rtu_poll_interval_ms_, [this]()
                         { poll_uart_response_(); });
      this->polling_active_ = true;
    }

    void ModbusBridgeComponent::stop_uart_polling_()
    {
      if (!this->polling_active_)
        return;
      this->cancel_interval("modbus_rx_poll");
      this->polling_active_ = false;
    }

    void ModbusBridgeComponent::poll_uart_response_()
    {
      if (this->pending_requests_.empty())
      {
        this->stop_uart_polling_();
        return;
      }
      PendingRequest &pending = this->pending_requests_.front();

      this->read_uart_response_bytes_(pending);

      size_t current_size = pending.response.size();

      if (current_size == 0)
      {
        uint32_t dt = millis() - pending.start_time;
        if (dt > this->rtu_response_timeout_ms_)
        {
          g_timeouts++;
          ESP_LOGW(TAG, "Modbus timeout: no response received (no first byte) client_id=%d", pending.client_fd);
          this->fire_rtu_timeout_for_request_(pending);
          this->finish_current_and_send_next_();
          return;
        }
        return; // still within overall timeout
      }

      // --- End-of-frame detection by size stability over consecutive polls ---
      // Consider the RTU response complete once we have observed no growth in
      // `pending.response.size()` for two consecutive polling intervals.
      if (current_size == pending.last_size) {
        if (pending.stable_polls < 255)
          pending.stable_polls++;
      } else {
        pending.last_size = current_size;
        pending.stable_polls = 0;
      }

      const uint8_t kStablePollsRequired = 2;
      if (pending.stable_polls >= kStablePollsRequired)
      {
        if (this->debug_)
        {
          std::string debug_output = to_hex(pending.response);
          ESP_LOGD(TAG, "RTU recv (stable %u polls, %d bytes): %s",
                   (unsigned)pending.stable_polls, (int)current_size, debug_output.c_str());
        }
        if (current_size < 3)
        {
          ESP_LOGW(TAG, "Invalid RTU response (<3 bytes) – dropping");
          this->finish_current_and_send_next_();
          return;
        }
        // Bridge-global event: RTU frame received (request context)
        {
          uint8_t fc = pdu_fc_from_rtu_(pending.rtu_data);
          uint16_t addr = start_addr_from_rtu_(pending.rtu_data);
          this->rtu_receive_cb_.call((int)fc, (int)addr);
        }
        std::vector<uint8_t> tcp_response;
        build_tcp_from_rtu(pending, pending.response, tcp_response);
        if (this->debug_)
        {
          std::string tcp_debug = to_hex(tcp_response);
          //ESP_LOGD(TAG, "RTU->TCP response: %s", tcp_debug.c_str());
          ESP_LOGD(TAG, "Response time: %ums", millis() - pending.start_time);
        }
        this->send_to_client_(pending.client_fd, tcp_response.data(), tcp_response.size());
        g_frames_out++;
        this->finish_current_and_send_next_();
        return;
      }

      if (millis() - pending.start_time > this->rtu_response_timeout_ms_)
      {
        g_timeouts++;
        ESP_LOGW(TAG, "Modbus timeout: response incomplete. Dropping. client_id=%d", pending.client_fd);
        INC(g_drops_rtu_incomplete);
        this->fire_rtu_timeout_for_request_(pending);
        this->finish_current_and_send_next_();
        return;
      }

      return;
    }

    void ModbusBridgeComponent::append_crc(std::vector<uint8_t> &data)
    {
      if (data.empty())
        return;
      uint16_t crc = modbus_crc(data.data(), data.size());
      if (this->crc_bytes_swapped_)
      {
        data.push_back((crc >> 8) & 0xFF);
        data.push_back(crc & 0xFF);
        return;
      }
      data.push_back(crc & 0xFF);
      data.push_back((crc >> 8) & 0xFF);
    }

    void ModbusBridgeComponent::set_debug(bool debug)
    {
      this->debug_ = debug;
      ESP_LOGI(TAG, "Debug mode %s", debug ? "enabled" : "disabled");
    }

    void ModbusBridgeComponent::set_enabled(bool enabled)
    {
      if (this->enabled_ == enabled)
        return;
      this->enabled_ = enabled;

      if (!enabled)
      {
        this->shutdown_tcp_and_pending_();
      }
      else
      {
        // Re-enabling: TCP server will be started again by tcp_server_and_network_check
      }
    }

    bool ModbusBridgeComponent::is_enabled() const
    {
      return this->enabled_;
    }

    // --- Runtime stats getters ----------------------------------------------------

    uint32_t ModbusBridgeComponent::get_frames_in() const { return g_frames_in; }
    uint32_t ModbusBridgeComponent::get_frames_out() const { return g_frames_out; }
    uint32_t ModbusBridgeComponent::get_drops_pid() const { return g_drops_pid; }
    uint32_t ModbusBridgeComponent::get_drops_tcp_len() const { return g_drops_tcp_len; }
    uint32_t ModbusBridgeComponent::get_drops_rtu_incomplete() const { return g_drops_rtu_incomplete; }
    uint32_t ModbusBridgeComponent::get_timeouts() const { return g_timeouts; }
    uint32_t ModbusBridgeComponent::get_clients_connected_total() const { return g_clients_connected; }
    uint32_t ModbusBridgeComponent::get_noslot_events() const { return g_noslot_events; }
    uint32_t ModbusBridgeComponent::get_preempt_events() const { return g_preempt_events; }

  } // namespace modbus_bridge
} // namespace esphome
