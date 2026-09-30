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
#include <lwip/tcpip.h>
#endif

#if defined(USE_ESP32) || defined(USE_ESP8266)
#include <lwip/dns.h>
#endif

#include "modbus_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/version.h"
#include "esphome/components/network/util.h"

namespace esphome
{
  namespace modbus_bridge
  {

    static const char *const TAG = "modbus_bridge";

    // Runtime-configurable (no build flags needed)
    static constexpr size_t kMaxPendingRequests = 32;
    static constexpr uint16_t MODBUS_TCP_LEN_CAP = 254;            // UID(1) + max PDU(253)
    static constexpr size_t MAX_TCP_READ = 6 + MODBUS_TCP_LEN_CAP; // MBAP(6) + LEN
    static constexpr size_t kTcpAccuCap = 2 * MAX_TCP_READ;        // partial frame + next TCP read
    static constexpr size_t kMaxRtuCapture = 512;                  // max RTU echo + response
    static constexpr size_t kMaxFramesPerLoop = 8;
    static constexpr size_t kMaxPendingRequestsPerUntrustedClient = 2;
    static constexpr uint32_t kTcpSendTimeoutMs = 10;
    static constexpr uint32_t kTrustedHostCacheMs = 60000;
    // Runtime toggle: preempt oldest same-IP connection when full
    static bool kPreemptSameIP = true;

    // Lightweight runtime counters aggregated across all bridge instances on the node.
    // This is intentional so diagnostics can reflect total bridge activity even when
    // multiple UART buses / bridge components are configured.

    static uint32_t g_frames_in = 0;
    static uint32_t g_frames_out = 0;
    static uint32_t g_drops_pid = 0;
    static uint32_t g_drops_tcp_len = 0;
    static uint32_t g_drops_rtu_incomplete = 0;
    static uint32_t g_drops_rtu_crc = 0;
    static uint32_t g_drops_rtu_mismatch = 0;
    static uint32_t g_drop_untrusted_reads = 0;
    static uint32_t g_drop_untrusted_writes = 0;
    static uint32_t g_drop_untrusted_queue = 0;
    static uint32_t g_reject_untrusted_clients = 0;
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

#ifdef USE_ESP8266
    static inline uint32_t ipv4_to_u32_(const IPAddress &ip)
    {
      return (static_cast<uint32_t>(ip[0]) << 24) |
             (static_cast<uint32_t>(ip[1]) << 16) |
             (static_cast<uint32_t>(ip[2]) << 8) |
             static_cast<uint32_t>(ip[3]);
    }
#endif

    static inline const char *ipv4_to_cstr_(uint32_t ip, char *buf, size_t len)
    {
      snprintf(buf, len, "%u.%u.%u.%u",
               static_cast<unsigned>((ip >> 24) & 0xFF),
               static_cast<unsigned>((ip >> 16) & 0xFF),
               static_cast<unsigned>((ip >> 8) & 0xFF),
               static_cast<unsigned>(ip & 0xFF));
      return buf;
    }

#if defined(USE_ESP32) || defined(USE_ESP8266)
    static void trusted_host_found_(const char *, const ip_addr_t *address, void *arg)
    {
      auto *lookup = static_cast<TrustedHostLookup *>(arg);
      lookup->ipv4 = address != nullptr && IP_IS_V4(address)
                         ? ntohl(ip4_addr_get_u32(ip_2_ip4(address)))
                         : 0;
      lookup->ready.store(true, std::memory_order_release);
    }

    static void start_trusted_host_lookup_(void *arg)
    {
      auto *lookup = static_cast<TrustedHostLookup *>(arg);
      ip_addr_t address;
      const err_t result = dns_gethostbyname_addrtype(lookup->hostname.c_str(), &address,
                                                     trusted_host_found_, arg, LWIP_DNS_ADDRTYPE_IPV4);
      if (result == ERR_OK)
        trusted_host_found_(nullptr, &address, arg);
      else if (result != ERR_INPROGRESS)
        trusted_host_found_(nullptr, nullptr, arg);
    }
#endif

    static void clear_rtu_response_(PendingRequest &req)
    {
      req.response.clear();
      req.last_size = 0;
      req.stable_polls = 0;
    }

    void ModbusBridgeComponent::drain_uart_rx_()
    {
      if (this->uart_ == nullptr)
        return;
      uint8_t b;
      bool received = false;
      // Bound each drain so a noisy bus cannot monopolize the main loop.
      const size_t available = this->uart_->available();
      for (size_t i = 0; i < available && this->uart_->read_byte(&b); ++i)
        received = true;
      if (received)
      {
        this->last_bus_activity_us_ = micros();
        this->bus_activity_seen_ = true;
      }
    }

    enum class BridgeUartFlushResult : uint8_t
    {
      SUCCESS,
      TIMEOUT,
      FAILED,
    };

    static inline BridgeUartFlushResult flush_uart_tx_(uart::UARTComponent *uart_component)
    {
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 4, 0)
      const auto result = uart_component->flush();
      if (result == uart::UARTFlushResult::UART_FLUSH_RESULT_SUCCESS ||
          result == uart::UARTFlushResult::UART_FLUSH_RESULT_ASSUMED_SUCCESS)
        return BridgeUartFlushResult::SUCCESS;
      if (result == uart::UARTFlushResult::UART_FLUSH_RESULT_TIMEOUT)
        return BridgeUartFlushResult::TIMEOUT;
      return BridgeUartFlushResult::FAILED;
#elif ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 3, 0)
      const auto result = uart_component->flush();
      if (result == uart::FlushResult::SUCCESS || result == uart::FlushResult::ASSUMED_SUCCESS)
        return BridgeUartFlushResult::SUCCESS;
      if (result == uart::FlushResult::TIMEOUT)
        return BridgeUartFlushResult::TIMEOUT;
      return BridgeUartFlushResult::FAILED;
#else
      uart_component->flush();
      return BridgeUartFlushResult::SUCCESS;
#endif
    }

    static inline const char *flush_result_to_cstr_(BridgeUartFlushResult result)
    {
      return result == BridgeUartFlushResult::TIMEOUT ? "timeout" : "driver failure";
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

    static inline uint16_t transaction_id_from_header_(const uint8_t header[7])
    {
      return (static_cast<uint16_t>(header[0]) << 8) | header[1];
    }

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

    static inline bool is_modbus_read_only_fc_(uint8_t fc)
    {
      switch (fc)
      {
      case 0x01:
      case 0x02:
      case 0x03:
      case 0x04:
        return true;
      default:
        return false;
      }
    }

    static inline bool has_known_response_shape_(uint8_t fc)
    {
      switch (fc)
      {
      case 0x01:
      case 0x02:
      case 0x03:
      case 0x04:
      case 0x05:
      case 0x06:
      case 0x0F:
      case 0x10:
        return true;
      default:
        return false;
      }
    }

    static inline bool write_response_echo_matches_request_(const PendingRequest &pending, const uint8_t *response, size_t len)
    {
      switch (pending.rtu_data[1])
      {
      case 0x05:
      case 0x06:
      case 0x0F:
      case 0x10:
        // Standard write responses echo start address plus value or quantity.
        return pending.rtu_data.size() >= 6 && len >= 6 &&
               std::equal(pending.rtu_data.begin() + 2, pending.rtu_data.begin() + 6, response + 2);
      default:
        return true;
      }
    }

    static inline bool request_expected_read_byte_count_(const PendingRequest &pending, uint8_t fc, uint8_t *byte_count)
    {
      if (byte_count == nullptr || pending.rtu_data.size() < 6)
        return false;

      const uint16_t quantity = (static_cast<uint16_t>(pending.rtu_data[4]) << 8) | pending.rtu_data[5];
      if (quantity == 0)
        return false;

      uint32_t expected = 0;
      switch (fc)
      {
      case 0x01:
      case 0x02:
        expected = (static_cast<uint32_t>(quantity) + 7U) / 8U;
        break;
      case 0x03:
      case 0x04:
        expected = static_cast<uint32_t>(quantity) * 2U;
        break;
      default:
        return false;
      }

      if (expected > 255U)
        return false;

      *byte_count = static_cast<uint8_t>(expected);
      return true;
    }

    static inline bool expected_known_rtu_response_length_(const PendingRequest &pending, size_t start, size_t *frame_len)
    {
      if (frame_len == nullptr || pending.rtu_data.size() < 2)
        return false;

      const auto &response = pending.response;
      if (start >= response.size() || response.size() - start < 2)
        return false;

      const uint8_t expected_fc = pending.rtu_data[1];
      const uint8_t response_fc = response[start + 1];

      if (response_fc == (expected_fc | 0x80))
      {
        *frame_len = 5;
        return true;
      }

      if (response_fc != expected_fc)
        return false;

      switch (response_fc)
      {
      case 0x01:
      case 0x02:
      case 0x03:
      case 0x04:
      {
        if (response.size() - start < 3)
          return false;
        uint8_t expected_byte_count = 0;
        if (!request_expected_read_byte_count_(pending, response_fc, &expected_byte_count))
          return false;
        if (response[start + 2] != expected_byte_count)
          return false;
        *frame_len = static_cast<size_t>(5U + expected_byte_count);
        return true;
      }
      case 0x05:
      case 0x06:
      case 0x0F:
      case 0x10:
        *frame_len = 8;
        return true;
      default:
        return false;
      }
    }

    void ProtectUntrustedReadsSwitch::write_state(bool state)
    {
      if (this->parent_ != nullptr)
        this->parent_->set_protect_reads_for_untrusted_clients(state);
      this->publish_state(state);
    }

    void ProtectUntrustedWritesSwitch::write_state(bool state)
    {
      if (this->parent_ != nullptr)
        this->parent_->set_protect_writes_for_untrusted_clients(state);
      this->publish_state(state);
    }

    void RejectUntrustedClientsSwitch::write_state(bool state)
    {
      if (this->parent_ != nullptr)
        this->parent_->set_reject_untrusted_clients(state);
      this->publish_state(state);
    }

    // Send one complete Modbus TCP response with a short bounded retry window.
    bool ModbusBridgeComponent::send_to_client_(int slot, const uint8_t *data, size_t len)
    {
      if (data == nullptr || len == 0 || slot < 0 || slot >= (int)this->clients_.size())
        return false;

#if defined(USE_ESP8266)
      auto &cl = this->clients_[slot];
      if (!cl.socket.connected())
        return false;

      size_t written = 0;
      const uint32_t started = millis();
      while (written < len && millis() - started < kTcpSendTimeoutMs)
      {
        const size_t n = cl.socket.write(data + written, len - written);
        if (n > 0)
          written += n;
        else
          delay(0);
      }
      if (written == len)
        return true;

      ESP_LOGW(TAG, "TCP send failed/short client_id=%d wrote=%u/%u", slot, (unsigned)written, (unsigned)len);
      cl.socket.stop();
      this->purge_client_((size_t)slot, &this->rx_accu8266_);
      this->refresh_tcp_client_count_();
      return false;
#elif defined(USE_ESP32)
      const int fd = this->clients_[slot].fd;
      if (fd < 0)
        return false;

      size_t written = 0;
      int last_error = 0;
      const uint32_t started = millis();
      while (written < len && millis() - started < kTcpSendTimeoutMs)
      {
        const int n = send(fd, data + written, len - written, 0);
        if (n > 0)
        {
          written += static_cast<size_t>(n);
          continue;
        }
        if (n < 0 && errno == EINTR)
          continue;
        if (n < 0 && errno != EWOULDBLOCK && errno != EAGAIN)
        {
          last_error = errno;
          break;
        }

        fd_set write_fds;
        FD_ZERO(&write_fds);
        FD_SET(fd, &write_fds);
        struct timeval timeout = {0, 2000}; // wait at most 2 ms before retrying
        const int selected = lwip_select(fd + 1, nullptr, &write_fds, nullptr, &timeout);
        if (selected < 0 && errno != EINTR)
        {
          last_error = errno;
          break;
        }
      }
      if (written == len)
        return true;

      ESP_LOGW(TAG, "TCP send failed/short client_id=%d wrote=%u/%u err=%s", slot,
               (unsigned)written, (unsigned)len, last_error != 0 ? strerror(last_error) : "timeout");
      this->purge_client_((size_t)slot, &this->rx_accu_);
      close(fd);
      this->clients_[slot].fd = -1;
      this->refresh_tcp_client_count_();
      return false;
#else
      return false;
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
      // Defensive cap for Modbus-TCP LEN (UID + PDU).
      while (accu.size() >= offset + 7)
      {
        if (processed++ >= (int)kMaxFramesPerLoop)
          break;
        const uint8_t *buf = accu.data() + offset;
        uint16_t len_field = static_cast<uint16_t>((buf[4] << 8) | buf[5]);

        // If LEN is clearly invalid, drop accumulator to recover from poison
        if (len_field < 2 || len_field > MODBUS_TCP_LEN_CAP)
        {
          ++g_drops_tcp_len;
          ESP_LOGW(TAG, "Invalid Modbus TCP length %u, dropping buffer client_id=%d", (unsigned)len_field, client_slot);
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

    template <typename Pred>
    static inline size_t find_first_slot_(size_t limit, Pred pred)
    {
      for (size_t i = 0; i < limit; ++i)
      {
        if (pred(i))
          return i;
      }
      return SIZE_MAX;
    }

    template <typename Pred, typename LastActivity>
    static inline size_t find_oldest_slot_(size_t limit, Pred pred, LastActivity last_activity)
    {
      size_t victim = SIZE_MAX;
      uint32_t oldest = 0xFFFFFFFFUL;
      for (size_t i = 0; i < limit; ++i)
      {
        if (!pred(i))
          continue;
        const uint32_t candidate = last_activity(i);
        if (victim == SIZE_MAX || candidate < oldest)
        {
          victim = i;
          oldest = candidate;
        }
      }
      return victim;
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

      auto it = this->pending_requests_.begin();
      if (this->rtu_request_active_ && it != this->pending_requests_.end() && it->client_fd == static_cast<int>(idx))
      {
        // The queue head has already been sent on RTU. Keep it until response or
        // timeout so late bytes cannot be mistaken for the next queued request.
        it->client_fd = -1;
        ++it;
      }

      // All remaining requests are unsent and can be removed, including a head
      // still waiting for the inter-frame gap.
      while (it != this->pending_requests_.end())
      {
        if (it->client_fd == static_cast<int>(idx))
          it = this->pending_requests_.erase(it);
        else
          ++it;
      }
      if (this->pending_requests_.empty())
      {
        this->cancel_timeout("modbus_tx");
        this->stop_uart_polling_();
      }
    }

    void ModbusBridgeComponent::record_tcp_client_connected_()
    {
      INC(g_clients_connected);
      this->update_tcp_client_count_();
    }

    void ModbusBridgeComponent::refresh_tcp_client_count_()
    {
      this->update_tcp_client_count_();
    }

    void ModbusBridgeComponent::prepare_rx_accumulator_(std::vector<std::vector<uint8_t>> &accu, size_t target_size, size_t reserve_cap)
    {
      if (accu.size() < target_size)
        accu.resize(target_size);
      for (auto &v : accu)
      {
        if (v.capacity() < reserve_cap)
          v.reserve(reserve_cap);
      }
    }

    void ModbusBridgeComponent::refresh_trusted_host_cache_()
    {
      if (this->trusted_hosts_resolving_)
        return;
      this->trusted_host_ipv4_cache_.clear();
      this->trusted_host_ipv4_cache_.reserve(this->trusted_hosts_.size());
      this->trusted_hosts_resolved_ = false;
      this->trusted_hosts_resolving_ = true;
      this->trusted_host_index_ = 0;
      this->poll_trusted_host_lookup_();
    }

    void ModbusBridgeComponent::poll_trusted_host_lookup_()
    {
      auto &lookup = this->trusted_host_lookup_;
      if (lookup.active)
      {
        if (!lookup.ready.load(std::memory_order_acquire))
          return;
        lookup.active = false;
        if (lookup.generation == this->trusted_hosts_generation_)
        {
          if (lookup.ipv4 != 0 && std::find(this->trusted_host_ipv4_cache_.begin(),
                                          this->trusted_host_ipv4_cache_.end(), lookup.ipv4) ==
                                      this->trusted_host_ipv4_cache_.end())
            this->trusted_host_ipv4_cache_.push_back(lookup.ipv4);
          ++this->trusted_host_index_;
        }
      }
      if (!this->trusted_hosts_resolving_)
        return;
      if (this->trusted_host_index_ >= this->trusted_hosts_.size())
      {
        this->trusted_hosts_resolving_ = false;
        this->trusted_hosts_resolved_ = true;
        this->trusted_hosts_last_resolve_ = millis();
        if (this->debug_)
          ESP_LOGD(TAG, "Resolved %u trusted host IPv4 address(es)", (unsigned)this->trusted_host_ipv4_cache_.size());
        return;
      }

      lookup.hostname = this->trusted_hosts_[this->trusted_host_index_];
      lookup.generation = this->trusted_hosts_generation_;
      lookup.ready.store(false, std::memory_order_relaxed);
      lookup.active = true;
#if defined(USE_ESP32)
      // Raw DNS calls must run on the lwIP task. Never wait for its mailbox or DNS.
      if (tcpip_try_callback(start_trusted_host_lookup_, &lookup) != ERR_OK)
        trusted_host_found_(nullptr, nullptr, &lookup);
#elif defined(USE_ESP8266)
      start_trusted_host_lookup_(&lookup);
#endif
    }

    bool ModbusBridgeComponent::is_trusted_client_ipv4_(uint32_t remote_ipv4)
    {
      for (const auto &net : this->trusted_networks_)
      {
        if ((remote_ipv4 & net.mask) == net.network)
          return true;
      }
      if (this->trusted_hosts_.empty())
        return false;

      if (!this->trusted_hosts_resolved_ || millis() - this->trusted_hosts_last_resolve_ >= kTrustedHostCacheMs)
        this->refresh_trusted_host_cache_();

      return this->trusted_hosts_resolved_ &&
             std::find(this->trusted_host_ipv4_cache_.begin(), this->trusted_host_ipv4_cache_.end(), remote_ipv4) !=
             this->trusted_host_ipv4_cache_.end();
    }

    bool ModbusBridgeComponent::finish_client_trust_check_(size_t slot)
    {
#if defined(USE_ESP32) || defined(USE_ESP8266)
      auto &client = this->clients_[slot];
      if (!client.trust_pending)
        return true;
      if (this->trusted_hosts_resolving_)
        return false;
      client.trust_pending = false;
      client.trusted = this->is_trusted_client_ipv4_(client.remote_ipv4);
      if (this->is_reject_untrusted_clients_effective_() && !client.trusted)
      {
        char ipbuf[16];
        ++g_reject_untrusted_clients;
        ESP_LOGW(TAG, "Rejecting untrusted TCP client %s:%u", ipv4_to_cstr_(client.remote_ipv4, ipbuf, sizeof(ipbuf)),
                 (unsigned)client.remote_port);
#if defined(USE_ESP32)
        this->purge_client_(slot, &this->rx_accu_);
        close(client.fd);
        client.fd = -1;
#else
        client.socket.stop();
        this->purge_client_(slot, &this->rx_accu8266_);
#endif
        this->refresh_tcp_client_count_();
        return false;
      }
      if (this->debug_)
        ESP_LOGD(TAG, "TCP trust resolved client_id=%u trusted=%s", (unsigned)slot, client.trusted ? "yes" : "no");
#endif
      return true;
    }

    bool ModbusBridgeComponent::is_write_protection_effective_() const
    {
      return this->protect_writes_for_untrusted_clients_ && this->has_trust_rules_();
    }

    bool ModbusBridgeComponent::is_read_protection_effective_() const
    {
      return this->protect_reads_for_untrusted_clients_ && this->has_trust_rules_();
    }

    bool ModbusBridgeComponent::is_reject_untrusted_clients_effective_() const
    {
      return this->reject_untrusted_clients_ && this->has_trust_rules_();
    }

    bool ModbusBridgeComponent::has_trust_rules_() const
    {
      return !this->trusted_networks_.empty() || !this->trusted_hosts_.empty();
    }

    void ModbusBridgeComponent::set_protect_untrusted_reads_switch(ProtectUntrustedReadsSwitch *sw)
    {
      this->protect_untrusted_reads_switch_ = sw;
      if (this->protect_untrusted_reads_switch_ != nullptr)
        this->protect_untrusted_reads_switch_->publish_state(this->protect_reads_for_untrusted_clients_);
    }

    void ModbusBridgeComponent::set_protect_untrusted_writes_switch(ProtectUntrustedWritesSwitch *sw)
    {
      this->protect_untrusted_writes_switch_ = sw;
      if (this->protect_untrusted_writes_switch_ != nullptr)
        this->protect_untrusted_writes_switch_->publish_state(this->protect_writes_for_untrusted_clients_);
    }

    void ModbusBridgeComponent::set_reject_untrusted_clients_switch(RejectUntrustedClientsSwitch *sw)
    {
      this->reject_untrusted_clients_switch_ = sw;
      if (this->reject_untrusted_clients_switch_ != nullptr)
        this->reject_untrusted_clients_switch_->publish_state(this->reject_untrusted_clients_);
    }

    void ModbusBridgeComponent::set_protect_reads_for_untrusted_clients(bool enabled)
    {
      this->protect_reads_for_untrusted_clients_ = enabled;
      if (this->protect_untrusted_reads_switch_ != nullptr && this->protect_untrusted_reads_switch_->state != enabled)
        this->protect_untrusted_reads_switch_->publish_state(enabled);
    }

    void ModbusBridgeComponent::set_protect_writes_for_untrusted_clients(bool enabled)
    {
      this->protect_writes_for_untrusted_clients_ = enabled;
      if (this->protect_untrusted_writes_switch_ != nullptr && this->protect_untrusted_writes_switch_->state != enabled)
        this->protect_untrusted_writes_switch_->publish_state(enabled);
    }

    void ModbusBridgeComponent::set_reject_untrusted_clients(bool enabled)
    {
      this->reject_untrusted_clients_ = enabled;
      if (this->reject_untrusted_clients_switch_ != nullptr &&
          this->reject_untrusted_clients_switch_->state != enabled)
        this->reject_untrusted_clients_switch_->publish_state(enabled);
    }

    void ModbusBridgeComponent::handle_client_rx_chunk_(std::vector<uint8_t> &accu, int client_fd, const uint8_t *data, size_t len, size_t max_accu)
    {
      if (len != 0)
        accu.insert(accu.end(), data, data + len);
      process_accu(accu, client_fd,
                   [&](const uint8_t *buf, size_t flen, int slot)
                   { handle_tcp_payload(buf, flen, slot); });
      if (accu.size() > max_accu)
        accu.clear();
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
      // The preceding UART flush guarantees that the final stop bit has left TX.
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
      // Above 19200 baud Modbus recommends a fixed 1.75 ms inter-frame gap.
      const uint32_t bits_per_char = std::max<uint32_t>(11, 1 + this->uart_->get_data_bits() +
          this->uart_->get_stop_bits() + (this->uart_->get_parity() != uart::UART_CONFIG_PARITY_NONE));
      this->rtu_frame_gap_us_ = _br_setup_guard > 19200 ? 1750 :
          static_cast<uint32_t>((bits_per_char * 3500000ULL + _br_setup_guard - 1) / _br_setup_guard);

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

      auto ips = network::get_ip_addresses();
      const bool have_ip = network::is_connected() &&
                          std::any_of(ips.begin(), ips.end(), [](const network::IPAddress &ip) {
                            return ip.is_ip4() && ip.is_set();
                          });

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
      this->tcp_response_buffer_.reserve(MAX_TCP_READ);
      this->rtu_poll_interval_ms_ = this->rtu_inactivity_timeout_ms_ + 2;
      this->stop_uart_polling_();
      this->tcp_client_count_ = 0;
      this->tcp_clients_changed_cb_.call(0);

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
                "stats: in=%u out=%u drops(pid)=%u drops(tcp_len)=%u drops(rtu_incomplete)=%u drops(rtu_crc)=%u drops(rtu_mismatch)=%u drop_untrusted_reads=%u drop_untrusted_writes=%u drop_untrusted_queue=%u reject_untrusted_clients=%u timeouts=%u clients_active=%u clients_total=%u noslot=%u preempt=%u",
                (unsigned)g_frames_in, (unsigned)g_frames_out, (unsigned)g_drops_pid,
                (unsigned)g_drops_tcp_len, (unsigned)g_drops_rtu_incomplete,
                (unsigned)g_drops_rtu_crc, (unsigned)g_drops_rtu_mismatch,
                (unsigned)g_drop_untrusted_reads, (unsigned)g_drop_untrusted_writes,
                (unsigned)g_drop_untrusted_queue, (unsigned)g_reject_untrusted_clients, (unsigned)g_timeouts,
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
      const auto ip = std::find_if(ips.begin(), ips.end(), [](const network::IPAddress &address) {
        return address.is_ip4() && address.is_set();
      });
      if (ip != ips.end())
      {
        char ipbuf[network::IP_ADDRESS_BUFFER_SIZE];
        ESP_LOGI(TAG, "TCP server started on %s:%d", ip->str_to(ipbuf), this->tcp_port_);
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
      // Let any outstanding callback finish before reusing its context, but ignore its result.
      ++this->trusted_hosts_generation_;
      this->trusted_hosts_resolving_ = false;
      this->trusted_hosts_resolved_ = false;
      this->trusted_host_ipv4_cache_.clear();
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
      this->rtu_request_active_ = false;
      this->cancel_timeout("modbus_tx");
      this->stop_uart_polling_();
      this->drain_uart_rx_();

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

    bool ModbusBridgeComponent::drop_protected_request_(uint8_t uid, uint8_t fc, int client_slot)
    {
      if (this->is_client_slot_trusted_(client_slot))
        return false;
      const bool protect_reads = this->is_read_protection_effective_();
      const bool protect_writes = this->is_write_protection_effective_();
      const bool read_only = is_modbus_read_only_fc_(fc);
      const char *reason = nullptr;
      if ((protect_reads || protect_writes) && uid == 0)
        reason = "broadcast UID 0";
      else if (protect_reads && read_only)
        reason = "read";
      else if (protect_writes && !read_only)
        reason = "unsafe function";
      if (reason == nullptr)
        return false;

      if (read_only)
        ++g_drop_untrusted_reads;
      else
        ++g_drop_untrusted_writes;
      if (this->debug_)
        ESP_LOGW(TAG, "Dropping untrusted Modbus %s FC 0x%02X from client_id=%d", reason, fc, client_slot);
      return true;
    }

    void ModbusBridgeComponent::handle_tcp_payload(const uint8_t *data, size_t len, int client_fd)
    {
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

      const uint8_t uid = data[6];
      const uint8_t fc = data[7];
      const bool trusted = this->is_client_slot_trusted_(client_fd);
      const bool protect_reads = this->is_read_protection_effective_();
      const bool protect_writes = this->is_write_protection_effective_();
      const bool protected_mode = protect_reads || protect_writes;
      if (this->drop_protected_request_(uid, fc, client_fd))
        return;

      if (!trusted && protected_mode)
      {
        size_t client_pending = 0;
        for (const auto &pending : this->pending_requests_)
        {
          if (pending.client_fd == client_fd)
            client_pending++;
        }
        if (client_pending >= kMaxPendingRequestsPerUntrustedClient)
        {
          g_drop_untrusted_queue++;
          if (this->debug_)
            ESP_LOGW(TAG, "Dropping untrusted Modbus request: client queue limit reached client_id=%d", client_fd);
          return;
        }
      }

      // Keep a full queue from being monopolized by untrusted clients. The
      // active queue head cannot be preempted, but a queued untrusted request can.
      if (this->pending_requests_.size() >= kMaxPendingRequests)
      {
        auto victim = this->pending_requests_.end();
        if (trusted && protected_mode && !this->pending_requests_.empty())
        {
          auto it = this->pending_requests_.begin();
          if (this->rtu_request_active_)
            ++it; // never remove a request already sent on RTU
          for (; it != this->pending_requests_.end(); ++it)
          {
            if (!it->trusted_client)
              victim = it;
          }
        }

        if (victim != this->pending_requests_.end())
        {
          this->pending_requests_.erase(victim);
          g_drop_untrusted_queue++;
          if (this->debug_)
            ESP_LOGW(TAG, "Dropping queued untrusted request to admit trusted client");
        }
        else
        {
          if (!trusted && protected_mode)
            g_drop_untrusted_queue++;
          ESP_LOGW(TAG, "Pending request queue full (%u), dropping frame", (unsigned)this->pending_requests_.size());
          return;
        }
      }

      PendingRequest req;
      req.client_fd = client_fd;
      req.trusted_client = trusted;
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
        ESP_LOGD(TAG, "TCP->RTU TID: 0x%04X, UID: %d, FC: 0x%02X, LEN: %d (client_id=%d, last activity %.3f s ago)",
                 transaction_id_from_header_(data),
                 uid,
                 req.rtu_data.size() > 1 ? req.rtu_data[1] : 0,
                 modbus_len, client_fd, seconds_ago);
      }

      memcpy(req.header, data, 7);
      req.start_time = 0;
      req.last_size = 0; // ensure deterministic timeout logic
      req.stable_polls = 0;
      g_frames_in++;
      if (trusted && protected_mode && !this->pending_requests_.empty())
      {
        auto insert_at = this->pending_requests_.begin();
        if (this->rtu_request_active_)
          ++insert_at; // a request already on the bus always remains first
        while (insert_at != this->pending_requests_.end() && insert_at->trusted_client)
          ++insert_at;
        this->pending_requests_.insert(insert_at, std::move(req));
      }
      else
      {
        this->pending_requests_.push_back(std::move(req));
      }

      this->dispatch_next_request_();
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
      const uint32_t remote_ipv4 = ipv4_to_u32_(new_client.remoteIP());
      const uint16_t remote_port = new_client.remotePort();
      const bool trusted = !this->has_trust_rules_() || this->is_trusted_client_ipv4_(remote_ipv4);
      const bool trust_pending = !trusted && this->trusted_hosts_resolving_;
      char ipbuf[16];
      if (this->is_reject_untrusted_clients_effective_() && !trusted && !trust_pending)
      {
        g_reject_untrusted_clients++;
        ESP_LOGW(TAG, "Rejecting untrusted TCP client %s:%u", ipv4_to_cstr_(remote_ipv4, ipbuf, sizeof(ipbuf)), (unsigned) remote_port);
        new_client.stop();
        return;
      }

      const size_t duplicate_idx = find_first_slot_(std::min(allowed_clients, this->clients_.size()), [&](size_t idx)
      {
        auto &ex = this->clients_[idx];
        return ex.socket.connected() &&
               ex.remote_ipv4 == remote_ipv4 &&
               ex.remote_port == remote_port;
      });
      if (duplicate_idx != SIZE_MAX)
      {
        auto &ex = this->clients_[duplicate_idx];
        ex.socket.stop();
        this->purge_client_(duplicate_idx, &this->rx_accu8266_);
        ex.socket = new_client;
        ex.socket.setNoDelay(true);
        ex.socket.setTimeout(10);
        ex.last_activity = millis();
        ex.disconnect_notified = false;
        ex.remote_ipv4 = remote_ipv4;
        ex.remote_port = remote_port;
        ex.trusted = trusted;
        ex.trust_pending = trust_pending;
        this->refresh_tcp_client_count_();
        return;
      }

      // Ensure accumulator size tracks clients_ size and reserve capacity
      this->prepare_rx_accumulator_(this->rx_accu8266_, this->clients_.size(), kTcpAccuCap);

      const size_t free_idx = find_first_slot_(std::min(allowed_clients, this->clients_.size()), [&](size_t idx)
      {
        return !this->clients_[idx].socket.connected();
      });
      if (free_idx != SIZE_MAX)
      {
        // The disconnected slot may still own queued state because accepts are
        // handled before disconnect cleanup on ESP8266.
        this->purge_client_(free_idx, &this->rx_accu8266_);
        this->clients_[free_idx].socket.stop();
        this->clients_[free_idx].socket = new_client;
        this->clients_[free_idx].socket.setNoDelay(true);
        this->clients_[free_idx].socket.setTimeout(10);
        this->clients_[free_idx].last_activity = millis();
        this->rx_accu8266_[free_idx].clear();
        this->clients_[free_idx].disconnect_notified = false;
        this->clients_[free_idx].remote_ipv4 = remote_ipv4;
        this->clients_[free_idx].remote_port = remote_port;
        this->clients_[free_idx].trusted = trusted;
        this->clients_[free_idx].trust_pending = trust_pending;
        ESP_LOGI(TAG, "TCP connect %s:%u client_id=%d trusted=%s",
                 ipv4_to_cstr_(remote_ipv4, ipbuf, sizeof(ipbuf)), (unsigned)remote_port, (int)free_idx,
                 trust_pending ? "pending" : (trusted ? "yes" : "no"));
        this->record_tcp_client_connected_();
        return;
      }

      if (this->clients_.size() < allowed_clients)
      {
        TCPClient8266 client;
        client.socket = new_client;
        client.socket.setNoDelay(true);
        client.socket.setTimeout(10);
        client.last_activity = millis();
        client.remote_ipv4 = remote_ipv4;
        client.remote_port = remote_port;
        client.trusted = trusted;
        client.trust_pending = trust_pending;
        this->clients_.push_back(client);
        this->rx_accu8266_.emplace_back();
        if (this->rx_accu8266_.back().capacity() < kTcpAccuCap)
          this->rx_accu8266_.back().reserve(kTcpAccuCap);
        this->clients_.back().disconnect_notified = false;
        ESP_LOGI(TAG, "TCP connect %s:%u client_id=%d trusted=%s",
                 ipv4_to_cstr_(remote_ipv4, ipbuf, sizeof(ipbuf)), (unsigned)remote_port,
                 (int)(this->clients_.size() - 1), trust_pending ? "pending" : (trusted ? "yes" : "no"));
        this->record_tcp_client_connected_();
        return;
      }

      bool preempted = false;
      if (kPreemptSameIP && !trust_pending)
      {
        const size_t victim = find_oldest_slot_(std::min(allowed_clients, this->clients_.size()),
                                                [&](size_t idx)
                                                {
                                                  auto &cl = this->clients_[idx];
                                                  return cl.socket.connected() && cl.remote_ipv4 == remote_ipv4;
                                                },
                                                [&](size_t idx)
                                                { return this->clients_[idx].last_activity; });
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
          this->clients_[victim].remote_ipv4 = remote_ipv4;
          this->clients_[victim].remote_port = remote_port;
          this->clients_[victim].trusted = trusted;
          this->clients_[victim].trust_pending = false;
          INC(g_preempt_events);
          this->record_tcp_client_connected_();
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

      const uint32_t remote_ipv4 = ntohl(client_addr.sin_addr.s_addr);
      const uint16_t remote_port = ntohs(client_addr.sin_port);
      const bool trusted = !this->has_trust_rules_() || this->is_trusted_client_ipv4_(remote_ipv4);
      const bool trust_pending = !trusted && this->trusted_hosts_resolving_;
      char client_ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
      if (this->debug_)
        ESP_LOGD(TAG, "TCP accept %s:%d", client_ip, remote_port);
      if (this->is_reject_untrusted_clients_effective_() && !trusted && !trust_pending)
      {
        g_reject_untrusted_clients++;
        ESP_LOGW(TAG, "Rejecting untrusted TCP client %s:%u", client_ip, (unsigned) remote_port);
        close(newfd);
        return;
      }

      configure_tcp_client_socket_(newfd);
      const size_t duplicate_idx = find_first_slot_(this->clients_.size(), [&](size_t idx)
      {
        auto &c = this->clients_[idx];
        return c.fd >= 0 && c.remote_ipv4 == remote_ipv4 && c.remote_port == remote_port;
      });
      if (duplicate_idx != SIZE_MAX)
      {
        auto &c = this->clients_[duplicate_idx];
        this->purge_client_(duplicate_idx, &this->rx_accu_);
        close(c.fd);
        c.fd = newfd;
        c.last_activity = millis();
        c.remote_ipv4 = remote_ipv4;
        c.remote_port = remote_port;
        c.trusted = trusted;
        c.trust_pending = trust_pending;
        this->refresh_tcp_client_count_();
        return;
      }

      const size_t free_idx = find_first_slot_(allowed_clients, [&](size_t idx)
      {
        return this->clients_[idx].fd < 0;
      });
      if (free_idx != SIZE_MAX)
      {
        auto &c = this->clients_[free_idx];
        // Keep slot reuse safe even if a previous close path left stale state.
        this->purge_client_(free_idx, &this->rx_accu_);
        c.fd = newfd;
        c.last_activity = millis();
        c.remote_ipv4 = remote_ipv4;
        c.remote_port = remote_port;
        c.trusted = trusted;
        c.trust_pending = trust_pending;
        ESP_LOGI(TAG, "TCP connect %s:%u client_id=%zu trusted=%s", client_ip, (unsigned)remote_port, free_idx,
                 trust_pending ? "pending" : (trusted ? "yes" : "no"));
        this->record_tcp_client_connected_();
        return;
      }

      bool preempted = false;
      if (kPreemptSameIP && !trust_pending)
      {
        const size_t victim = find_oldest_slot_(allowed_clients,
                                                [&](size_t idx)
                                                {
                                                  auto &c = this->clients_[idx];
                                                  return c.fd >= 0 && c.remote_ipv4 == remote_ipv4;
                                                },
                                                [&](size_t idx)
                                                { return this->clients_[idx].last_activity; });
        if (victim != SIZE_MAX)
        {
          this->purge_client_(victim, &this->rx_accu_);
          close(this->clients_[victim].fd);
          this->clients_[victim].fd = newfd;
          this->clients_[victim].last_activity = millis();
          this->clients_[victim].remote_ipv4 = remote_ipv4;
          this->clients_[victim].remote_port = remote_port;
          this->clients_[victim].trusted = trusted;
          this->clients_[victim].trust_pending = false;
          INC(g_preempt_events);
          this->record_tcp_client_connected_();
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
      this->poll_trusted_host_lookup_();
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
      this->prepare_rx_accumulator_(this->rx_accu8266_, this->clients_.size(), kTcpAccuCap);
      if (this->sock_ < 0)
      {
        for (auto &v : this->rx_accu8266_)
          v.clear();
        return;
      }
      const size_t allowed_clients = this->tcp_allowed_clients_;
      const int server_socket = this->sock_;
      const auto poll_valid = [this, server_socket, allowed_clients]() {
        return this->enabled_ && this->sock_ == server_socket && this->tcp_allowed_clients_ == allowed_clients;
      };
      this->handle_new_client_esp8266_(allowed_clients);
      if (!poll_valid())
        return;

      for (auto it = this->clients_.begin(); it != this->clients_.end();)
      {
        const size_t idx = static_cast<size_t>(std::distance(this->clients_.begin(), it));
        if (idx >= allowed_clients)
        {
          it->socket.stop();
          it->disconnect_notified = true;
          this->purge_client_(idx, &this->rx_accu8266_);
          this->refresh_tcp_client_count_();
          if (!poll_valid())
            return;
          ++it;
          continue;
        }
        if (!it->socket.connected())
        {
          // keep slot, log only once per transition
          if (!it->disconnect_notified)
          {
            ESP_LOGI(TAG, "TCP disconnect client_id=%u", (unsigned)idx);
            it->disconnect_notified = true;
          }
          it->socket.stop();
          this->purge_client_(idx, &this->rx_accu8266_);
          this->refresh_tcp_client_count_();
          if (!poll_valid())
            return;
          ++it;
          continue;
        }

        if (millis() - it->last_activity > this->tcp_client_timeout_ms_)
        {
          ESP_LOGW(TAG, "TCP timeout client_id=%u", (unsigned)idx);
          it->socket.stop();
          it->disconnect_notified = true;
          this->purge_client_(idx, &this->rx_accu8266_);
          this->refresh_tcp_client_count_();
          if (!poll_valid())
            return;
          ++it;
          continue;
        }

        const size_t client_fd = static_cast<size_t>(std::distance(this->clients_.begin(), it));
        if (it->trust_pending && it->socket.available() > 0)
        {
          auto &accu = this->rx_accu8266_[client_fd];
          const size_t room = kTcpAccuCap - accu.size();
          const int received = it->socket.read(this->temp_buffer_.data(), std::min(this->temp_buffer_.size(), room + 1));
          if (received > 0 && static_cast<size_t>(received) > room)
          {
            ESP_LOGW(TAG, "TCP trust-pending buffer full client_id=%u", (unsigned)client_fd);
            it->socket.stop();
            this->purge_client_(client_fd, &this->rx_accu8266_);
            this->refresh_tcp_client_count_();
            if (!poll_valid())
              return;
            ++it;
            continue;
          }
          if (received > 0)
          {
            accu.insert(accu.end(), this->temp_buffer_.begin(), this->temp_buffer_.begin() + received);
            it->last_activity = millis();
          }
        }
        const bool trust_ready = this->finish_client_trust_check_(client_fd);
        if (!poll_valid())
          return;
        if (!trust_ready)
        {
          ++it;
          continue;
        }
        auto &accu = this->rx_accu8266_[client_fd];
        const size_t room = accu.size() < kTcpAccuCap ? kTcpAccuCap - accu.size() : 0;
        const size_t to_read = std::min({static_cast<size_t>(it->socket.available()), this->temp_buffer_.size(), room});
        const int r = to_read > 0 ? it->socket.read(this->temp_buffer_.data(), to_read) : 0;
        if (r > 0)
        {
          it->disconnect_notified = false;
          it->last_activity = millis();
        }
        // Also drain complete buffered frames when the peer sends no new bytes.
        this->handle_client_rx_chunk_(accu, static_cast<int>(client_fd), this->temp_buffer_.data(),
                                     r > 0 ? static_cast<size_t>(r) : 0, kTcpAccuCap);
        if (!poll_valid())
          return;

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
      if (this->sock_ < 0)
      {
        for (auto &v : this->rx_accu_)
          v.clear();
        return; // keep accepting/reading even when requests are pending
      }

      const size_t allowed_clients = this->tcp_allowed_clients_;
      const int server_socket = this->sock_;
      const auto poll_valid = [this, server_socket, allowed_clients]() {
        return this->enabled_ && this->sock_ == server_socket && this->tcp_allowed_clients_ == allowed_clients;
      };
      if (this->clients_.size() < allowed_clients)
        this->clients_.resize(allowed_clients);
      this->prepare_rx_accumulator_(this->rx_accu_, this->clients_.size(), kTcpAccuCap);

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
            this->purge_client_(idx, &this->rx_accu_);
            close(c.fd);
            c.fd = -1;
            this->refresh_tcp_client_count_();
            if (!poll_valid())
              return;
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
            this->refresh_tcp_client_count_();
            if (!poll_valid())
              return;
            continue;
          }
          if (c.trust_pending)
          {
            // Buffer without dispatching so EOF remains visible even after the peer sent data.
            auto &accu = this->rx_accu_[idx];
            const size_t room = kTcpAccuCap - accu.size();
            const int received = recv(c.fd, this->temp_buffer_.data(), std::min(this->temp_buffer_.size(), room + 1), 0);
            const bool overflow = received > 0 && static_cast<size_t>(received) > room;
            if (overflow || received == 0 ||
                (received < 0 && errno != EWOULDBLOCK && errno != EAGAIN && errno != EINTR))
            {
              if (overflow)
                ESP_LOGW(TAG, "TCP trust-pending buffer full client_id=%zu", idx);
              else
                ESP_LOGI(TAG, "TCP disconnect while trust pending client_id=%zu", idx);
              this->purge_client_(idx, &this->rx_accu_);
              close(c.fd);
              c.fd = -1;
              this->refresh_tcp_client_count_();
              if (!poll_valid())
                return;
              continue;
            }
            if (received > 0)
            {
              accu.insert(accu.end(), this->temp_buffer_.begin(), this->temp_buffer_.begin() + received);
              c.last_activity = now;
            }
          }
          const bool trust_ready = this->finish_client_trust_check_(idx);
          if (!poll_valid())
            return;
          if (!trust_ready)
            continue;
          FD_SET(c.fd, &read_fds);
          if (c.fd > maxfd)
            maxfd = c.fd;
        }
      }

      // This method is already called periodically; do not block the ESPHome loop.
      struct timeval timeout = {0, 0};
      int sel = lwip_select(maxfd + 1, &read_fds, NULL, NULL, &timeout);
      if (sel < 0)
        return;

      if (FD_ISSET(this->sock_, &read_fds))
        this->handle_new_client_esp32_(allowed_clients);
      if (!poll_valid())
        return;

      for (size_t i = 0; i < allowed_clients; ++i)
      {
        auto &c = this->clients_[i];
        if (c.fd < 0 || c.trust_pending)
          continue;
        auto &accu = this->rx_accu_[i];
        const size_t room = accu.size() < kTcpAccuCap ? kTcpAccuCap - accu.size() : 0;
        int r = 0;
        if (FD_ISSET(c.fd, &read_fds) && room > 0)
        {
          r = recv(c.fd, this->temp_buffer_.data(), std::min(this->temp_buffer_.size(), room), 0);
          if (r == 0)
          {
            ESP_LOGI(TAG, "TCP disconnect client_id=%zu", i);
            this->purge_client_(i, &this->rx_accu_);
            close(c.fd);
            c.fd = -1;
            this->refresh_tcp_client_count_();
            if (!poll_valid())
              return;
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
              this->refresh_tcp_client_count_();
              if (!poll_valid())
                return;
              continue;
            }
          }
          if (r > 0)
            c.last_activity = now;
        }
        this->handle_client_rx_chunk_(accu, static_cast<int>(i), this->temp_buffer_.data(),
                                     r > 0 ? static_cast<size_t>(r) : 0, kTcpAccuCap);
        if (!poll_valid())
          return;
      }
#endif
    }

    bool ModbusBridgeComponent::is_client_slot_connected_(int slot)
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

    bool ModbusBridgeComponent::is_client_slot_trusted_(int slot) const
    {
      if (slot < 0 || slot >= (int)this->clients_.size())
        return false;
      return this->clients_[slot].trusted;
    }

    bool ModbusBridgeComponent::send_rtu_request_(PendingRequest &req)
    {
      clear_rtu_response_(req);
      req.received_bytes = false;
      if (req.response.capacity() < MAX_TCP_READ)
        req.response.reserve(MAX_TCP_READ);

      BridgeUartFlushResult flush_result = flush_uart_tx_(this->uart_);
      if (flush_result != BridgeUartFlushResult::SUCCESS)
      {
        ESP_LOGE(TAG, "UART flush %s before RTU send. client_id=%d tid=0x%04X",
                 flush_result_to_cstr_(flush_result), req.client_fd,
                 transaction_id_from_header_(req.header));
        return false;
      }
      this->rs485_begin_tx_();
      this->uart_->write_array(req.rtu_data);
      flush_result = flush_uart_tx_(this->uart_);
      this->rs485_end_tx_();
      this->last_bus_activity_us_ = micros();
      this->bus_activity_seen_ = true;
      if (flush_result != BridgeUartFlushResult::SUCCESS)
      {
        ESP_LOGE(TAG, "UART flush %s after RTU write; transmission may be incomplete. client_id=%d tid=0x%04X",
                 flush_result_to_cstr_(flush_result), req.client_fd,
                 transaction_id_from_header_(req.header));
        return false;
      }
      req.start_time = millis();

      if (this->debug_)
      {
        ESP_LOGD(TAG, "RTU send: %s client_id=%d tid=0x%04X",
                 to_hex(req.rtu_data).c_str(), req.client_fd, transaction_id_from_header_(req.header));
      }

      uint8_t fc = pdu_fc_from_rtu_(req.rtu_data);
      uint16_t addr = start_addr_from_rtu_(req.rtu_data);
      this->rtu_send_cb_.call((int)fc, (int)addr);
      return true;
    }

    void ModbusBridgeComponent::abort_pending_after_uart_tx_failure_()
    {
      const size_t dropped = this->pending_requests_.size();
      this->pending_requests_.clear();
      this->rtu_request_active_ = false;
      this->cancel_timeout("modbus_tx");
      this->stop_uart_polling_();
      this->drain_uart_rx_();
      ESP_LOGW(TAG, "Aborted %u pending Modbus request(s) after UART TX failure", (unsigned)dropped);
    }

    bool ModbusBridgeComponent::finish_current_and_send_next_()
    {
      if (this->rtu_request_active_ && !this->pending_requests_.empty())
        this->pending_requests_.pop_front();
      this->rtu_request_active_ = false;
      this->stop_uart_polling_();
      return this->dispatch_next_request_();
    }

    bool ModbusBridgeComponent::dispatch_next_request_()
    {
      if (this->rtu_request_active_)
        return true;
      this->cancel_timeout("modbus_tx");
      while (!this->pending_requests_.empty())
      {
        auto &next = this->pending_requests_.front();
        if (!this->is_client_slot_connected_(next.client_fd) ||
            this->drop_protected_request_(next.rtu_data[0], next.rtu_data[1], next.client_fd))
        {
          this->pending_requests_.pop_front();
          continue;
        }

        // Observe/drain late bytes before starting a new transaction. UART APIs
        // do not expose their wire timestamp, so use the last observed activity.
        this->drain_uart_rx_();
        const uint32_t idle_us = micros() - this->last_bus_activity_us_;
        if (this->bus_activity_seen_ && idle_us < this->rtu_frame_gap_us_)
        {
          const uint32_t wait_ms = (this->rtu_frame_gap_us_ - idle_us + 999) / 1000;
          this->set_timeout("modbus_tx", wait_ms, [this]() { this->dispatch_next_request_(); });
          return true;
        }

        this->rtu_request_active_ = true;
        if (!this->send_rtu_request_(next))
        {
          this->abort_pending_after_uart_tx_failure_();
          return false;
        }
        if (!this->rtu_request_active_ || this->pending_requests_.empty())
        {
          // An on_rtu_send automation may disable the bridge and clear the queue.
          this->stop_uart_polling_();
          return false;
        }
        this->start_uart_polling_();
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

    void ModbusBridgeComponent::check_rtu_timeout_(const PendingRequest &req)
    {
      if (millis() - req.start_time <= this->rtu_response_timeout_ms_)
        return;
      ++g_timeouts;
      if (!req.response.empty())
      {
        ESP_LOGW(TAG, "Incomplete RTU response (%d bytes): %s", (int)req.response.size(), to_hex(req.response).c_str());
        ESP_LOGW(TAG, "Modbus timeout: response incomplete. Dropping. client_id=%d", req.client_fd);
        ++g_drops_rtu_incomplete;
      }
      else if (req.received_bytes)
        ESP_LOGW(TAG, "Modbus timeout: no valid matching response received client_id=%d", req.client_fd);
      else
        ESP_LOGW(TAG, "Modbus timeout: no response received (no first byte) client_id=%d", req.client_fd);
      this->fire_rtu_timeout_for_request_(req);
      this->finish_current_and_send_next_();
    }

    size_t ModbusBridgeComponent::read_uart_response_bytes_(PendingRequest &req)
    {
      size_t avail = this->uart_->available();
      if (!avail)
        return 0;

      const size_t remaining = req.response.size() < kMaxRtuCapture
                                   ? kMaxRtuCapture - req.response.size()
                                   : 0;
      req.response.reserve(req.response.size() + std::min(avail, remaining));
      size_t discarded = 0;
      bool received = false;
      for (size_t i = 0; i < avail; ++i)
      {
        uint8_t b;
        if (this->uart_->read_byte(&b))
        {
          req.received_bytes = true;
          received = true;
          if (req.response.size() < kMaxRtuCapture)
            req.response.push_back(b);
          else
            discarded++;
        }
        else
          break;
      }
      if (received)
      {
        this->last_bus_activity_us_ = micros();
        this->bus_activity_seen_ = true;
      }
      return discarded;
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
      if (!this->rtu_request_active_ || this->pending_requests_.empty())
      {
        this->stop_uart_polling_();
        return;
      }
      PendingRequest &pending = this->pending_requests_.front();

      const size_t discarded = this->read_uart_response_bytes_(pending);
      if (discarded > 0)
      {
        INC(g_drops_rtu_incomplete);
        ESP_LOGW(TAG,
                 "RTU response exceeded capture limit (%u bytes, at least %u discarded). Dropping. client_id=%d bytes=%s",
                 (unsigned)kMaxRtuCapture, (unsigned)discarded, pending.client_fd,
                 to_hex(pending.response).c_str());
        clear_rtu_response_(pending);
        this->check_rtu_timeout_(pending);
        return;
      }

      size_t current_size = pending.response.size();

      if (current_size == 0)
      {
        this->check_rtu_timeout_(pending);
        return;
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
      size_t expected_len = 0;
      const bool known_length = expected_known_rtu_response_length_(pending, 0, &expected_len);
      const bool complete_known_frame = known_length && current_size == expected_len &&
                                        this->validate_rtu_response_matches_request_(pending) &&
                                        this->validate_rtu_crc_(pending.response);

      // Clean responses take the fast path. Search each unchanged noisy buffer
      // once after it settles, and once more at the original deadline.
      const bool timed_out = millis() - pending.start_time > this->rtu_response_timeout_ms_;
      if (complete_known_frame || pending.stable_polls == kStablePollsRequired || timed_out)
      {
        bool incomplete = false;
        const bool normalized = !complete_known_frame && this->normalize_rtu_response_(pending, incomplete);
        // A misleading echo header must not hide a complete response later in
        // the buffer. If no complete candidate exists, retain partial data.
        if (!complete_known_frame && !normalized &&
            (incomplete || (timed_out && pending.stable_polls < kStablePollsRequired)))
        {
          this->check_rtu_timeout_(pending);
          return;
        }
        current_size = pending.response.size();
        if (this->debug_)
        {
          std::string debug_output = to_hex(pending.response);
          if (complete_known_frame)
            ESP_LOGD(TAG, "RTU recv (complete known frame, %d bytes): %s", (int)current_size, debug_output.c_str());
          else
            ESP_LOGD(TAG, "RTU recv (stable %u polls, %d bytes): %s",
                     (unsigned)pending.stable_polls, (int)current_size, debug_output.c_str());
        }
        bool crc_valid = complete_known_frame || normalized;
        if (current_size < 5)
        {
          INC(g_drops_rtu_incomplete);
          ESP_LOGW(TAG, "Invalid RTU response (<5 bytes) – dropping");
          clear_rtu_response_(pending);
          this->check_rtu_timeout_(pending);
          return;
        }
        if (!crc_valid)
          crc_valid = this->validate_rtu_crc_(pending.response);
        if (!crc_valid)
        {
          INC(g_drops_rtu_crc);
          ESP_LOGW(TAG, "RTU CRC mismatch. Dropping response. client_id=%d bytes=%s",
                   pending.client_fd, to_hex(pending.response).c_str());
          clear_rtu_response_(pending);
          this->check_rtu_timeout_(pending);
          return;
        }
        if (!complete_known_frame && !normalized && !this->validate_rtu_response_matches_request_(pending))
        {
          INC(g_drops_rtu_mismatch);
          ESP_LOGW(TAG, "RTU response does not match request. Dropping response. client_id=%d req_uid=%u req_fc=0x%02X bytes=%s",
                   pending.client_fd,
                   pending.rtu_data.size() > 0 ? (unsigned)pending.rtu_data[0] : 0U,
                   pending.rtu_data.size() > 1 ? pending.rtu_data[1] : 0,
                   to_hex(pending.response).c_str());
          clear_rtu_response_(pending);
          this->check_rtu_timeout_(pending);
          return;
        }
        const uint8_t event_fc = pdu_fc_from_rtu_(pending.rtu_data);
        const uint16_t event_addr = start_addr_from_rtu_(pending.rtu_data);
        if (this->is_client_slot_connected_(pending.client_fd))
        {
          auto &tcp_response = this->tcp_response_buffer_;
          build_tcp_from_rtu(pending, pending.response, tcp_response);
          if (this->send_to_client_(pending.client_fd, tcp_response.data(), tcp_response.size()))
          {
            g_frames_out++;
            if (this->debug_)
            {
              ESP_LOGD(TAG, "RTU->TCP TID: 0x%04X, LEN: %u, Response time: %ums",
                       transaction_id_from_header_(pending.header),
                       tcp_response.size() >= 6 ? static_cast<unsigned>((tcp_response[4] << 8) | tcp_response[5]) : 0U,
                       static_cast<unsigned>(millis() - pending.start_time));
            }
          }
        }
        else if (this->debug_)
        {
          ESP_LOGD(TAG, "Discarding RTU response for disconnected client, TID: 0x%04X",
                   transaction_id_from_header_(pending.header));
        }
        // Run automations only after all request data has been consumed. An
        // automation may disable the bridge and clear pending_requests_.
        this->rtu_receive_cb_.call((int)event_fc, (int)event_addr);
        this->finish_current_and_send_next_();
        return;
      }

      this->check_rtu_timeout_(pending);
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

    bool ModbusBridgeComponent::validate_rtu_crc_(const std::vector<uint8_t> &data) const
    {
      return this->validate_rtu_crc_(data.data(), data.size());
    }

    bool ModbusBridgeComponent::validate_rtu_crc_(const uint8_t *data, size_t len) const
    {
      if (data == nullptr || len < 4)
        return false;

      const size_t payload_len = len - 2;
      const uint16_t calculated = modbus_crc(data, payload_len);
      uint16_t received = 0;

      if (this->crc_bytes_swapped_)
      {
        received = (static_cast<uint16_t>(data[payload_len]) << 8) |
                   static_cast<uint16_t>(data[payload_len + 1]);
      }
      else
      {
        received = static_cast<uint16_t>(data[payload_len]) |
                   (static_cast<uint16_t>(data[payload_len + 1]) << 8);
      }

      return calculated == received;
    }

    bool ModbusBridgeComponent::validate_rtu_response_matches_request_(const PendingRequest &pending) const
    {
      return this->validate_rtu_response_matches_request_(pending, 0, pending.response.size());
    }

    bool ModbusBridgeComponent::validate_rtu_response_matches_request_(const PendingRequest &pending, size_t start, size_t len) const
    {
      if (pending.rtu_data.size() < 2 || len < 5 || start > pending.response.size() || len > pending.response.size() - start)
        return false;
      const uint8_t *response = pending.response.data() + start;

      const uint8_t request_uid = pending.rtu_data[0];
      const uint8_t request_fc = pending.rtu_data[1];
      const uint8_t response_uid = response[0];
      const uint8_t response_fc = response[1];

      if (response_uid != request_uid)
        return false;
      if (response_fc != request_fc && response_fc != (request_fc | 0x80))
        return false;

      size_t expected_len = 0;
      if (expected_known_rtu_response_length_(pending, start, &expected_len))
      {
        if (expected_len != len)
          return false;
        if (response_fc == (request_fc | 0x80))
          return true;
        return write_response_echo_matches_request_(pending, response, len);
      }
      if (has_known_response_shape_(request_fc))
        return false;

      // Unknown function codes stay transparent; after UID/FC and CRC checks,
      // there is no response layout we can safely validate further.
      return true;
    }

    bool ModbusBridgeComponent::normalize_rtu_response_(PendingRequest &pending, bool &incomplete)
    {
      incomplete = false;
      auto &response = pending.response;
      if (pending.rtu_data.size() < 2 || response.size() < 2)
        return false;

      const uint8_t expected_uid = pending.rtu_data[0];
      for (size_t start = 0; start + 2 <= response.size(); ++start)
      {
        if (response[start] != expected_uid)
          continue;

        size_t frame_len = 0;
        if (!expected_known_rtu_response_length_(pending, start, &frame_len))
        {
          if (response.size() - start < 3 && response[start + 1] == pending.rtu_data[1] &&
              has_known_response_shape_(pending.rtu_data[1]))
            incomplete = true;
          continue;
        }
        if (frame_len > response.size() - start)
        {
          incomplete = true;
          continue;
        }
        if (!this->validate_rtu_response_matches_request_(pending, start, frame_len))
          continue;
        if (!this->validate_rtu_crc_(response.data() + start, frame_len))
          continue;

        const size_t leading = start;
        const size_t trailing = response.size() - start - frame_len;
        if (leading == 0 && trailing == 0)
          return true;

        if (trailing > 0)
          response.erase(response.begin() + start + frame_len, response.end());
        if (leading > 0)
          response.erase(response.begin(), response.begin() + leading);

        if (this->debug_)
        {
          ESP_LOGD(TAG, "RTU echo/noise stripped: leading=%u trailing=%u response=%s",
                   (unsigned)leading, (unsigned)trailing, to_hex(response).c_str());
        }
        return true;
      }

      return false;
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

    // --- Runtime stats getters (node-wide aggregated counters) -------------------

    uint32_t ModbusBridgeComponent::get_frames_in() const { return g_frames_in; }
    uint32_t ModbusBridgeComponent::get_frames_out() const { return g_frames_out; }
    uint32_t ModbusBridgeComponent::get_drops_pid() const { return g_drops_pid; }
    uint32_t ModbusBridgeComponent::get_drops_tcp_len() const { return g_drops_tcp_len; }
    uint32_t ModbusBridgeComponent::get_drops_rtu_incomplete() const { return g_drops_rtu_incomplete; }
    uint32_t ModbusBridgeComponent::get_drops_rtu_crc() const { return g_drops_rtu_crc; }
    uint32_t ModbusBridgeComponent::get_drops_rtu_mismatch() const { return g_drops_rtu_mismatch; }
    uint32_t ModbusBridgeComponent::get_drop_untrusted_reads() const { return g_drop_untrusted_reads; }
    uint32_t ModbusBridgeComponent::get_drop_untrusted_writes() const { return g_drop_untrusted_writes; }
    uint32_t ModbusBridgeComponent::get_drop_untrusted_queue() const { return g_drop_untrusted_queue; }
    uint32_t ModbusBridgeComponent::get_reject_untrusted_clients() const { return g_reject_untrusted_clients; }
    uint32_t ModbusBridgeComponent::get_timeouts() const { return g_timeouts; }
    uint32_t ModbusBridgeComponent::get_clients_connected_total() const { return g_clients_connected; }
    uint32_t ModbusBridgeComponent::get_noslot_events() const { return g_noslot_events; }
    uint32_t ModbusBridgeComponent::get_preempt_events() const { return g_preempt_events; }

  } // namespace modbus_bridge
} // namespace esphome
