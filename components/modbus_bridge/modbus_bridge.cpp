#include <vector>
#include <memory>
#include <functional>
#include <initializer_list>
#include <cstring>
#include <cmath>
#include <array>

#ifdef USE_ARDUINO
#include <Arduino.h>
#include "IPAddress.h"
#endif


#ifdef USE_ESP32
#include <lwip/sockets.h>
#include <fcntl.h>
#endif

#include "modbus_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/application.h"
#include "esphome/components/network/util.h"


namespace esphome {
namespace modbus_bridge {

static const char *const TAG = "modbus_bridge";

// Tunables (override via -D flags if needed)
#ifndef MODBUS_BRIDGE_MAX_PENDING_REQUESTS
#define MODBUS_BRIDGE_MAX_PENDING_REQUESTS 32
#endif
#ifndef MODBUS_BRIDGE_TCP_ACCU_CAP
#define MODBUS_BRIDGE_TCP_ACCU_CAP 1024
#endif
#ifndef MODBUS_BRIDGE_TCP_ACCU_CAP_8266
#define MODBUS_BRIDGE_TCP_ACCU_CAP_8266 1024
#endif
#ifndef MODBUS_BRIDGE_MAX_FRAMES_PER_LOOP
#define MODBUS_BRIDGE_MAX_FRAMES_PER_LOOP 8
#endif

#if defined(USE_ESP8266)
static constexpr size_t MAX_TCP_CLIENTS = 2;  // leaner on RAM for ESP8266
#else
static constexpr size_t MAX_TCP_CLIENTS = 4;
#endif

// Lightweight runtime counters (file-scope, assume single instance)

static uint32_t g_frames_in = 0;
static uint32_t g_frames_out = 0;
static uint32_t g_drops_pid = 0;
static uint32_t g_drops_len = 0;
static uint32_t g_timeouts = 0;
static uint32_t g_clients_connected = 0;

// --- Helpers to reduce duplication ---------------------------------------------------------

// Cheap hex conversion for debug logs
static inline std::string to_hex(const uint8_t *data, size_t n) {
  std::string s; s.reserve(n * 3);
  for (size_t i = 0; i < n; ++i) { char t[4]; sprintf(t, "%02X ", data[i]); s += t; }
  return s;
}
static inline std::string to_hex(const std::vector<uint8_t> &v) {
  return to_hex(v.data(), v.size());
}

// Drain UART RX (templated to avoid pulling specific UART headers here)
template <typename U>
static inline void drain_uart_rx(U *u) {
  uint8_t b; while (u && u->available()) u->read_byte(&b);
}

using FrameHandler = std::function<void(const uint8_t*, size_t, int)>;

// Build Modbus TCP response from an RTU response
static inline void build_tcp_from_rtu(const PendingRequest &pending,
                                      const std::vector<uint8_t> &rtu_resp,
                                      std::vector<uint8_t> &out) {
  const size_t current_size = rtu_resp.size();
  const uint16_t pdu_length = static_cast<uint16_t>(current_size - 3); // FC + Data
  const uint16_t mbap_len   = static_cast<uint16_t>(pdu_length + 1);   // UID + PDU
  out.clear();
  out.reserve(6 + 2 + 1 + pdu_length);                                 // MBAP + LEN + UID + PDU
  out.insert(out.end(), pending.header, pending.header + 4);           // TID + PID
  out.push_back((mbap_len >> 8) & 0xFF);
  out.push_back(mbap_len & 0xFF);
  out.push_back(rtu_resp[0]);                                          // UID
  out.insert(out.end(), rtu_resp.begin() + 1, rtu_resp.end() - 2);     // PDU (FC+Data), drop CRC
}


// Member method to unify sending to a client slot on both platforms
void ModbusBridgeComponent::send_to_client_(int slot, const uint8_t *data, size_t len) {
#if defined(USE_ESP8266)
  if (slot >= 0 && slot < (int)this->clients_.size()) {
    auto &cl = this->clients_[slot];
    if (cl.socket.connected()) {
      cl.socket.write(data, len);
    }
  }
#elif defined(USE_ESP32)
  if (slot >= 0 && slot < (int)MAX_TCP_CLIENTS) {
    int fd = this->clients_[slot].fd;
    if (fd >= 0) {
      send(fd, data, len, 0);
    }
  }
#endif
}

// Extract and process as many complete Modbus-TCP frames as possible from an accumulator
static inline void process_accu(std::vector<uint8_t> &accu, int client_slot, const FrameHandler &on_frame) {
  int processed = 0;
  while (accu.size() >= 7) {
    if (processed++ >= MODBUS_BRIDGE_MAX_FRAMES_PER_LOOP) break;
    const uint8_t *buf = accu.data();
    uint16_t len_field = static_cast<uint16_t>((buf[4] << 8) | buf[5]);
    size_t frame_len = 6UL + static_cast<size_t>(len_field); // MBAP(6) + LEN (UID+PDU)
    if (accu.size() < frame_len) break;                      // incomplete
    on_frame(buf, frame_len, client_slot);
    accu.erase(accu.begin(), accu.begin() + frame_len);
  }
}

// Tiny counter helper
#ifndef INC
#define INC(x) do { ++(x); } while (0)
#endif
// -------------------------------------------------------------------------------------------


ModbusBridgeComponent::ModbusBridgeComponent() {}


void ModbusBridgeComponent::set_rtu_response_timeout(uint32_t timeout) {
  this->rtu_response_timeout_ms_ = timeout;
}

void ModbusBridgeComponent::setup() {
  this->sock_ = -1;

  this->set_interval("tcp_server_and_network_check", 1000, [this]() {
    if (this->sock_ < 0 && network::is_connected()) {
      ESP_LOGI(TAG, "IP available – initializing TCP server");
      this->initialize_tcp_server_();
    } else if (this->sock_ >= 0 && !network::is_connected()) {
      ESP_LOGW(TAG, "Lost network IP – closing TCP server");
#if defined(USE_ESP8266)
      this->server_.stop();
#elif defined(USE_ESP32)
      close(this->sock_);
#endif
      this->sock_ = -1;
      // Also close all active clients and clear per-client state
#if defined(USE_ESP8266)
      for (auto &cl : this->clients_) {
        if (cl.socket.connected()) cl.socket.stop();
      }
      // Accumulators are static in check_tcp_sockets_() and will be cleared there when sock_ < 0
#elif defined(USE_ESP32)
      for (auto &cl : this->clients_) {
        if (cl.fd >= 0) { close(cl.fd); cl.fd = -1; }
      }
      // Accumulators are static in check_tcp_sockets_() and will be cleared there when sock_ < 0
#endif
    }
  });

  this->set_interval("tcp_poll", this->tcp_poll_interval_ms_, [this]() {
    this->check_tcp_sockets_();
  });

  // t3.5 ≈ 3.5 character times, conservatively 11 bits/char
  this->rtu_inactivity_timeout_ms_ = static_cast<uint32_t>(std::ceil((3.5 * 11.0 * 1000.0) / this->uart_->get_baud_rate()) + 1);
  // Use a fixed-size scratch buffer for TCP reads (independent of UART RX size)
  // 512 bytes comfortably covers typical Modbus-TCP frames (MBAP + PDU)
  this->temp_buffer_.resize(512);
  this->rtu_poll_interval_ms_ = this->rtu_inactivity_timeout_ms_ + 2;
  this->polling_active_ = false;
  if (this->rtu_response_timeout_ms_ == 0) {
    ESP_LOGW(TAG, "RTU response timeout set to 0 – falling back to 100 ms.");
    this->rtu_response_timeout_ms_ = 100;
  }

  // Periodic status log (debug only)
  this->set_interval("status_log", 10'000, [this]() {
    if (!this->debug_) return;
    size_t clients_active = 0;
  #if defined(USE_ESP8266)
    for (auto &cl : this->clients_) if (cl.socket.connected()) clients_active++;
  #elif defined(USE_ESP32)
    for (auto &cl : this->clients_) if (cl.fd >= 0) clients_active++;
  #endif
    ESP_LOGD(TAG,
            "stats: in=%u out=%u drops(pid)=%u drops(len)=%u timeouts=%u clients_active=%u clients_total=%u",
            (unsigned)g_frames_in, (unsigned)g_frames_out, (unsigned)g_drops_pid,
            (unsigned)g_drops_len, (unsigned)g_timeouts,
            (unsigned)clients_active, (unsigned)g_clients_connected);
  });
}


void ModbusBridgeComponent::initialize_tcp_server_() {
#if defined(USE_ESP8266)
  this->server_ = WiFiServer(this->tcp_port_);
  this->server_.begin();
  this->sock_ = 1;  // Dummywert für „Server läuft“
  ESP_LOGI(TAG, "TCP server started on %s:%d", WiFi.localIP().toString().c_str(), this->tcp_port_);
#elif defined(USE_ESP32)
  this->sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (this->sock_ < 0) {
    ESP_LOGE(TAG, "Socket creation failed");
    return;
  }

  fcntl(this->sock_, F_SETFL, O_NONBLOCK);

  // No TCP_NODELAY on server socket itself (set on accept)

  struct sockaddr_in server_addr = {};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(this->tcp_port_);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  bind(this->sock_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  listen(this->sock_, MAX_TCP_CLIENTS);

  auto ips = network::get_ip_addresses();
  if (!ips.empty()) {
    ESP_LOGI(TAG, "TCP server started on %s:%d", ips[0].str().c_str(), this->tcp_port_);
  } else {
    ESP_LOGI(TAG, "TCP server started, but no IP address found");
  }

  for (auto &c : this->clients_) c.fd = -1;
#endif
}

void ModbusBridgeComponent::handle_tcp_payload(const uint8_t *data, size_t len, int client_fd) {
  // DoS protection: cap pending requests
  if (this->pending_requests_.size() >= MODBUS_BRIDGE_MAX_PENDING_REQUESTS) {
    ESP_LOGW(TAG, "Pending request queue full (%u), dropping frame", (unsigned)this->pending_requests_.size());
    return;
  }
  if (len < 7) {
    ESP_LOGW(TAG, "Received too-short frame (%d bytes)", (int)len);
    g_drops_len++;
    return;
  }

  // MBAP sanity check: Protocol ID must be 0
  if (data[2] != 0 || data[3] != 0) {
    ESP_LOGW(TAG, "Non-zero Protocol ID, dropping frame");
    g_drops_pid++;
    return;
  }

  uint16_t modbus_len = (data[4] << 8) | data[5];
  // LEN counts UID + PDU; require at least UID(1)+FC(1)
  if (modbus_len < 2) {
    ESP_LOGW(TAG, "Invalid Modbus length (<2), dropping frame: %u", (unsigned)modbus_len);
    g_drops_len++;
    return;
  }
  // Hard cap to prevent abuse / oversized frames (typical max ~260 bytes)
  const uint16_t MODBUS_LEN_CAP = 260;
  if (modbus_len > MODBUS_LEN_CAP) {
    ESP_LOGW(TAG, "Modbus length too large (%u > %u), dropping frame", (unsigned)modbus_len, (unsigned)MODBUS_LEN_CAP);
    g_drops_len++;
    return;
  }
  // Function code must be non-zero (first byte of PDU)
  if (len >= 8 && data[7] == 0x00) {
    ESP_LOGW(TAG, "Invalid function code 0x00, dropping frame");
    g_drops_pid++;
    return;
  }
  if (modbus_len > this->temp_buffer_.size() - 6) {
    ESP_LOGW(TAG, "Invalid Modbus length field: %d", modbus_len);
    g_drops_len++;
    return;
  }
  if (len < 6 + modbus_len) return;

  uint8_t uid = data[6];
  std::vector<uint8_t> rtu;
  rtu.reserve(static_cast<size_t>(modbus_len) + 1 + 2); // UID + PDU + CRC
  rtu.push_back(uid);
  rtu.insert(rtu.end(), data + 7, data + 6 + modbus_len);
  append_crc(rtu);

  if (this->debug_) {
    uint32_t now = millis();
    uint32_t last = this->clients_[client_fd].last_activity;
    float seconds_ago = (now - last) / 1000.0f;
    ESP_LOGD(TAG, "TCP->RTU UID: %d, FC: 0x%02X, LEN: %d (client_id=%d, last activity %.3f s ago)",
         uid, rtu[1], modbus_len, client_fd, seconds_ago);
  }

  PendingRequest req;
  req.client_fd = client_fd;
  req.rtu_data = rtu;
  memcpy(req.header, data, 7);
  req.response.reserve(this->uart_->get_rx_buffer_size());
  req.start_time = millis();
  req.last_change = millis();
  g_frames_in++;
  this->pending_requests_.push_back(std::move(req));

  if (this->pending_requests_.size() == 1) {
    if (this->debug_) {
      ESP_LOGD(TAG, "RTU send: %s client_id=%d", to_hex(rtu).c_str(), client_fd);
    }
    this->uart_->flush();
    // Drain any stale RX bytes before sending a new RTU request
    drain_uart_rx(this->uart_);
    this->uart_->write_array(rtu);
    this->start_uart_polling_();
  }
}

void ModbusBridgeComponent::check_tcp_sockets_() {
 #if defined(USE_ESP8266)
  static std::vector<std::vector<uint8_t>> rx_accu8266; // per-slot accumulator
  if (this->sock_ < 0) {
    for (auto &v : rx_accu8266) v.clear();
  }
  WiFiClient new_client = this->server_.accept();
  if (new_client) {
    // Duplicate connection check: same IP and port
    bool duplicate = false;
    for (size_t idx = 0; idx < this->clients_.size(); ++idx) {
      auto &ex = this->clients_[idx];
      if (!ex.socket.connected()) continue;
      if (ex.socket.remoteIP() == new_client.remoteIP() && ex.socket.remotePort() == new_client.remotePort()) {
        ESP_LOGW(TAG, "Duplicate connection from %s:%u – closing new client", new_client.remoteIP().toString().c_str(), (unsigned)new_client.remotePort());
        new_client.stop();
        duplicate = true;
        break;
      }
    }
    if (duplicate) {
      // skip further processing for this new_client
    } else {
      // Ensure accumulator size tracks clients_ size
      if (rx_accu8266.size() < this->clients_.size()) rx_accu8266.resize(this->clients_.size());

      // Try to reuse a free slot (disconnected client)
      bool placed = false;
      for (size_t idx = 0; idx < this->clients_.size(); ++idx) {
        if (!this->clients_[idx].socket.connected()) {
          this->clients_[idx].socket.stop();
          this->clients_[idx].socket = new_client;
          this->clients_[idx].socket.setNoDelay(true);
          this->clients_[idx].socket.setTimeout(10);
          this->clients_[idx].last_activity = millis();
          rx_accu8266[idx].clear();
          this->clients_[idx].disconnect_notified = false;
          ESP_LOGI(TAG, "TCP connect %s:%u client_id=%d", new_client.remoteIP().toString().c_str(), (unsigned)new_client.remotePort(), (int)idx);
          g_clients_connected++;
          placed = true;
          break;
        }
      }
      // If no free slot, append if under MAX
      if (!placed) {
        if (this->clients_.size() < MAX_TCP_CLIENTS) {
          TCPClient8266 client;
          client.socket = new_client;
          client.socket.setNoDelay(true);
          client.socket.setTimeout(10);
          client.last_activity = millis();
          this->clients_.push_back(client);
          rx_accu8266.emplace_back();
          this->clients_.back().disconnect_notified = false;
          ESP_LOGI(TAG, "TCP connect %s:%u client_id=%d", new_client.remoteIP().toString().c_str(), (unsigned)new_client.remotePort(), (int)(this->clients_.size() - 1));
          g_clients_connected++;
        } else {
          new_client.stop();
          ESP_LOGW(TAG, "No slot for new TCP client, closing connection");
        }
      }
    } // end duplicate check else
  }

  for (auto it = this->clients_.begin(); it != this->clients_.end(); ) {
    if (!it->socket.connected()) {
      // keep slot, log only once per transition
      size_t idx = static_cast<size_t>(std::distance(this->clients_.begin(), it));
      if (!it->disconnect_notified) {
        ESP_LOGI(TAG, "TCP disconnect client_id=%u", (unsigned)idx);
        it->disconnect_notified = true;
      }
      it->socket.stop();
      if (idx < rx_accu8266.size()) rx_accu8266[idx].clear();
      ++it;
      continue;
    }

    if (millis() - it->last_activity > this->tcp_client_timeout_ms_) {
      size_t idx = static_cast<size_t>(std::distance(this->clients_.begin(), it));
      ESP_LOGW(TAG, "TCP timeout client_id=%u", (unsigned)idx);
      it->socket.stop();
      it->disconnect_notified = true;  // suppress duplicate disconnect log afterwards
      if (idx < rx_accu8266.size()) rx_accu8266[idx].clear();
      ++it;
      continue;
    }

    if (it->socket.available() >= 1) {
      int r = it->socket.readBytes(this->temp_buffer_.data(), this->temp_buffer_.size());
      int client_fd = static_cast<int>(std::distance(this->clients_.begin(), it));
      if (rx_accu8266.size() < this->clients_.size()) rx_accu8266.resize(this->clients_.size());
      // Append incoming data
      rx_accu8266[client_fd].insert(rx_accu8266[client_fd].end(), this->temp_buffer_.begin(), this->temp_buffer_.begin() + r);
      process_accu(rx_accu8266[client_fd], client_fd,
                   [&](const uint8_t *buf, size_t flen, int slot){ handle_tcp_payload(buf, flen, slot); });
      // Cap accumulator size to avoid growth on garbage
      const size_t kMaxAccu8266 = MODBUS_BRIDGE_TCP_ACCU_CAP_8266;
      if (rx_accu8266[client_fd].size() > kMaxAccu8266) rx_accu8266[client_fd].clear();
      this->clients_[client_fd].disconnect_notified = false; // receiving traffic confirms connection
      it->last_activity = millis();
    }

    ++it;
  }
 #elif defined(USE_ESP32)
  static std::array<std::vector<uint8_t>, MAX_TCP_CLIENTS> rx_accu; // per-client TCP accumulator
  if (this->sock_ < 0) {
    for (auto &v : rx_accu) v.clear();
    return; // keep accepting/reading even when requests are pending
  }

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(this->sock_, &read_fds);
  int maxfd = this->sock_;

  uint32_t now = millis();

  for (auto &c : this->clients_) {
    if (c.fd >= 0) {
      if (now - c.last_activity > this->tcp_client_timeout_ms_) {
          ESP_LOGW(TAG, "TCP timeout client_id=%zu", (size_t)(&c - &this->clients_[0]));
        size_t idx_timeout = static_cast<size_t>(&c - &this->clients_[0]);
        if (idx_timeout < rx_accu.size()) rx_accu[idx_timeout].clear();
        close(c.fd);
        c.fd = -1;
        continue;
      }
      FD_SET(c.fd, &read_fds);
      if (c.fd > maxfd) maxfd = c.fd;
    }
  }

  struct timeval timeout = {0, 0};
  int sel = select(maxfd + 1, &read_fds, NULL, NULL, &timeout);
  if (sel < 0) return;

  if (FD_ISSET(this->sock_, &read_fds)) {
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int newfd = accept(this->sock_, (struct sockaddr *)&client_addr, &addr_len);
    if (newfd >= 0) {
      char client_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
      ESP_LOGD(TAG, "TCP accept %s:%d", client_ip, ntohs(client_addr.sin_port));

      // Check for duplicates: same IP and port
      bool duplicate = false;
      for (auto &c : this->clients_) {
        if (c.fd < 0) continue;

        struct sockaddr_in existing_addr;
        socklen_t len = sizeof(existing_addr);
        if (getpeername(c.fd, (struct sockaddr *)&existing_addr, &len) == 0) {
          if (existing_addr.sin_addr.s_addr == client_addr.sin_addr.s_addr &&
              existing_addr.sin_port == client_addr.sin_port) {
            ESP_LOGW(TAG, "Duplicate connection from %s:%d – closing new connection", client_ip, ntohs(client_addr.sin_port));
            close(newfd);
            duplicate = true;
            break;
          }
        }
      }
      if (duplicate)
        return;

      fcntl(newfd, F_SETFL, O_NONBLOCK);
      int one = 1;
      setsockopt(newfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
      // Enable TCP keepalive (helps clean up dead peers)
      int ka = 1; setsockopt(newfd, SOL_SOCKET, SO_KEEPALIVE, &ka, sizeof(ka));
#ifdef TCP_KEEPIDLE
      int idle = 30; setsockopt(newfd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
#endif
#ifdef TCP_KEEPINTVL
      int intvl = 10; setsockopt(newfd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
#endif
#ifdef TCP_KEEPCNT
      int cnt = 3; setsockopt(newfd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
      bool accepted = false;
      for (auto &c : this->clients_) {
        if (c.fd < 0) {
          c.fd = newfd;
          c.last_activity = millis();
          ESP_LOGI(TAG, "TCP connect %s:%d client_id=%zu", client_ip, ntohs(client_addr.sin_port), (size_t)(&c - &this->clients_[0]));
          g_clients_connected++;
          accepted = true;
          break;
        }
      }
      if (!accepted) {
        ESP_LOGW(TAG, "No slot for new TCP client, closing connection");
        close(newfd);
      }
    }
  }

  for (size_t i = 0; i < MAX_TCP_CLIENTS; ++i) {
    auto &c = this->clients_[i];
    if (c.fd >= 0 && FD_ISSET(c.fd, &read_fds)) {
      int r = recv(c.fd, this->temp_buffer_.data(), this->temp_buffer_.size(), 0);
      if (r == 0) {
        ESP_LOGI(TAG, "TCP disconnect client_id=%zu", i);
        if (i < rx_accu.size()) rx_accu[i].clear();
        close(c.fd);
        c.fd = -1;
        continue;
      }
      if (r < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
          ESP_LOGW(TAG, "TCP error client_id=%zu err=%s", i, strerror(errno));
          if (i < rx_accu.size()) rx_accu[i].clear();
          close(c.fd);
          c.fd = -1;
        }
        continue;
      }
      if (r > 0) {
        size_t idx = i;
        // Append incoming bytes to accumulator
        rx_accu[idx].insert(rx_accu[idx].end(), this->temp_buffer_.begin(), this->temp_buffer_.begin() + r);
        process_accu(rx_accu[idx], static_cast<int>(idx),
                     [&](const uint8_t *buf, size_t flen, int slot){ handle_tcp_payload(buf, flen, slot); });
        // Prevent unbounded growth in pathological cases (drop oldest data)
        const size_t kMaxAccu = MODBUS_BRIDGE_TCP_ACCU_CAP;
        if (rx_accu[idx].size() > kMaxAccu) {
          rx_accu[idx].clear();
        }
        c.last_activity = now;
      }
    }
  }
#endif
}

void ModbusBridgeComponent::start_uart_polling_() {
  if (this->polling_active_) return;
  this->set_interval("modbus_rx_poll", this->rtu_poll_interval_ms_, [this]() { poll_uart_response_(); });
  this->polling_active_ = true;
}

void ModbusBridgeComponent::poll_uart_response_() {
  if (this->pending_requests_.empty()) {
    polling_active_ = false;
    return;
  }
  PendingRequest &pending = this->pending_requests_.front();

  auto finish_request = [&]() {
    this->pending_requests_.pop_front();
    if (!this->pending_requests_.empty()) {
      auto &next = this->pending_requests_.front();
      this->uart_->flush();
      // Drain any stale RX bytes before sending the next RTU request
      drain_uart_rx(this->uart_);
      this->uart_->write_array(next.rtu_data);
      if (this->debug_) {
        ESP_LOGD(TAG, "RTU send: %s client_id=%d", to_hex(next.rtu_data).c_str(), next.client_fd);
      }
    } else {
      this->polling_active_ = false;
    }
  };

  while (this->uart_->available()) {
    uint8_t byte;
    if (this->uart_->read_byte(&byte)) {
      pending.response.push_back(byte);
    }
  }

  size_t current_size = pending.response.size();

  if (current_size == 0) {
    if (millis() - pending.start_time > this->rtu_response_timeout_ms_) {
      g_timeouts++;
      ESP_LOGW(TAG, "Modbus timeout: no response received (no first byte) client_id=%d", pending.client_fd);
      finish_request();
      return;
    }
    return;
  }

  if (current_size < 3) {
    ESP_LOGW(TAG, "Invalid response length: too short");
    finish_request();
    return;
  }

  // Track changes in received data per request to reset timeout correctly
  if (current_size != pending.last_size) {  // requires: size_t last_size in PendingRequest
    pending.last_size = current_size;
    pending.last_change = millis();
  }

  if (millis() - pending.last_change > this->rtu_inactivity_timeout_ms_) {
    if (this->debug_) {
      std::string debug_output = to_hex(pending.response);
      ESP_LOGD(TAG, "RTU recv (%d bytes): %s", (int)current_size, debug_output.c_str());
    }

    std::vector<uint8_t> tcp_response;
    build_tcp_from_rtu(pending, pending.response, tcp_response);

    if (this->debug_) {
      std::string tcp_debug = to_hex(tcp_response);
      ESP_LOGD(TAG, "RTU->TCP response: %s", tcp_debug.c_str());
      ESP_LOGD(TAG, "Response time: %ums", millis() - pending.start_time);
    }

    this->send_to_client_(pending.client_fd, tcp_response.data(), tcp_response.size());
    g_frames_out++;
    finish_request();
    return;
  }

  if (millis() - pending.start_time > this->rtu_response_timeout_ms_) {
    g_timeouts++;
    ESP_LOGW(TAG, "Modbus timeout: response incomplete. client_id=%d", pending.client_fd);
    finish_request();
    return;
  }

  return;
}

// end_pending_request_ is now obsolete

void ModbusBridgeComponent::append_crc(std::vector<uint8_t> &data) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; i++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  data.push_back(crc & 0xFF);
  data.push_back((crc >> 8) & 0xFF);
}

void ModbusBridgeComponent::set_debug(bool debug) {
  this->debug_ = debug;
  ESP_LOGI(TAG, "Debug mode %s", debug ? "enabled" : "disabled");
}


}  // namespace modbus_bridge
}  // namespace esphome
