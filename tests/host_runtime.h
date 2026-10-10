#pragma once

// Only the hardware/framework boundary is simulated; tests compile the complete bridge.
#include <algorithm>
#include <array>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <string>
#include <vector>
#include <sys/socket.h>
#include <sys/select.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

inline void checked_fd_set(int fd, fd_set *set) {
  assert(fd >= 0 && fd < FD_SETSIZE); FD_SET(fd, set);
}
inline int checked_fd_isset(int fd, const fd_set *set) {
  assert(fd >= 0 && fd < FD_SETSIZE); return FD_ISSET(fd, set);
}
#undef FD_SET
#undef FD_ISSET
#define FD_SET(fd, set) checked_fd_set(fd, set)
#define FD_ISSET(fd, set) checked_fd_isset(fd, set)

namespace fake {
inline uint32_t now = 0;
inline uint32_t sub_ms = 0;
struct Connection {
  bool connected = true;
  bool listener = false;
  uint32_t ip = 0xC0A80120;
  uint16_t port = 40000;
  std::deque<uint8_t> rx;
  std::vector<uint8_t> tx;
};
inline std::map<int, Connection> connections;
inline std::deque<int> accepts;
inline int next_fd = 3;
inline int client(uint32_t ip = 0xC0A80120) {
  const int fd = next_fd++;
  connections[fd].ip = ip;
  connections[fd].port += fd;
  accepts.push_back(fd);
  return fd;
}
inline int socket(int, int, int) {
  const int fd = next_fd++;
  connections[fd].listener = true;
  return fd;
}
inline int close(int fd) { connections[fd].connected = false; return 0; }
inline int bind(int, const sockaddr *, socklen_t) { return 0; }
inline int listen(int, int) { return 0; }
inline int fcntl(int, int, int) { return 0; }
inline int setsockopt(int, int, int, const void *, socklen_t) { return 0; }
inline int accept(int, sockaddr *address, socklen_t *) {
  if (accepts.empty()) { errno = EAGAIN; return -1; }
  int fd = accepts.front(); accepts.pop_front();
  auto *peer = reinterpret_cast<sockaddr_in *>(address);
  peer->sin_addr.s_addr = htonl(connections[fd].ip);
  peer->sin_port = htons(connections[fd].port);
  return fd;
}
inline int recv(int fd, void *data, size_t len, int) {
  auto &c = connections[fd];
  if (c.rx.empty()) {
    if (!c.connected) return 0;
    errno = EAGAIN; return -1;
  }
  size_t n = std::min(len, c.rx.size());
  for (size_t i = 0; i < n; ++i) {
    static_cast<uint8_t *>(data)[i] = c.rx.front(); c.rx.pop_front();
  }
  return static_cast<int>(n);
}
inline int send(int fd, const void *data, size_t len, int) {
  auto &c = connections[fd];
  if (!c.connected) { errno = EPIPE; return -1; }
  const auto *p = static_cast<const uint8_t *>(data);
  c.tx.insert(c.tx.end(), p, p + len);
  return static_cast<int>(len);
}
inline int select(int limit, fd_set *reads, fd_set *, fd_set *, timeval *) {
  int count = 0;
  for (int fd = 0; reads && fd < limit; ++fd) {
    if (!FD_ISSET(fd, reads)) continue;
    const auto &c = connections[fd];
    bool ready = c.listener ? !accepts.empty() : (!c.connected || !c.rx.empty());
    if (ready) ++count; else FD_CLR(fd, reads);
  }
  return count;
}
inline std::vector<std::string> logs;
inline void log(const char *message) { logs.emplace_back(message); }
template<class... Args> void log(const char *format, Args... args) {
  char buf[2048]; snprintf(buf, sizeof(buf), format, args...); logs.emplace_back(buf);
}
}

#define socket(...) fake::socket(__VA_ARGS__)
#define close(...) fake::close(__VA_ARGS__)
#define bind(...) fake::bind(__VA_ARGS__)
#define listen(...) fake::listen(__VA_ARGS__)
#define fcntl(...) fake::fcntl(__VA_ARGS__)
#define setsockopt(...) fake::setsockopt(__VA_ARGS__)
#define accept(...) fake::accept(__VA_ARGS__)
#define recv(...) fake::recv(__VA_ARGS__)
#define send(...) fake::send(__VA_ARGS__)
#define lwip_select(...) fake::select(__VA_ARGS__)
#define ESP_LOGD(tag, ...) fake::log(__VA_ARGS__)
#define ESP_LOGI(tag, ...) fake::log(__VA_ARGS__)
#define ESP_LOGW(tag, ...) fake::log(__VA_ARGS__)
#define ESP_LOGE(tag, ...) fake::log(__VA_ARGS__)
#define VERSION_CODE(a,b,c) (((a) << 16) | ((b) << 8) | (c))
#define ESPHOME_VERSION_CODE VERSION_CODE(2026,8,2)

struct ip_addr_t { uint32_t addr = 0; };
#define IP_IS_V4(address) true
#define ip_2_ip4(address) (address)
#define ip4_addr_get_u32(address) ((address)->addr)
using err_t = int;
constexpr int ERR_OK = 0, ERR_INPROGRESS = -5, ERR_MEM = -1;
constexpr int LWIP_DNS_ADDRTYPE_IPV4 = 0;
using dns_found_callback = void (*)(const char *, const ip_addr_t *, void *);
namespace fake {
struct DNSRequest { std::string host; dns_found_callback callback; void *arg; };
inline std::deque<DNSRequest> dns_requests;
inline std::deque<std::function<void()>> dns_tasks;
inline std::map<std::string, uint32_t> dns_cached;
inline bool dns_error = false, dns_queue_full = false;
inline void run_dns_task() {
  if (dns_tasks.empty()) return;
  auto task = dns_tasks.front(); dns_tasks.pop_front(); task();
}
inline void dns_reply(uint32_t ip) {
  assert(!dns_requests.empty());
  auto req = dns_requests.front(); dns_requests.pop_front();
  ip_addr_t address{htonl(ip)};
  req.callback(req.host.c_str(), ip ? &address : nullptr, req.arg);
}
}
inline err_t dns_gethostbyname_addrtype(const char *host, ip_addr_t *address,
                                      dns_found_callback cb, void *arg, int) {
  if (fake::dns_error) return ERR_MEM;
  auto found = fake::dns_cached.find(host);
  if (found != fake::dns_cached.end()) { address->addr = htonl(found->second); return ERR_OK; }
  fake::dns_requests.push_back({host, cb, arg}); return ERR_INPROGRESS;
}
inline err_t tcpip_try_callback(void (*cb)(void *), void *arg) {
  if (fake::dns_queue_full) return ERR_MEM;
  fake::dns_tasks.push_back([cb,arg]() { cb(arg); }); return ERR_OK;
}

namespace esphome {
inline uint32_t millis() { return fake::now; }
inline uint32_t micros() { return fake::now * 1000U + fake::sub_ms; }
inline void delay(unsigned value) { fake::now += value; }
inline void delayMicroseconds(unsigned value) {
  const uint32_t total = fake::sub_ms + value;
  fake::now += total / 1000; fake::sub_ms = total % 1000;
}
class Component {
public:
  std::map<std::string, std::function<void()>> intervals;
  struct Timeout { uint32_t due; std::function<void()> callback; };
  std::map<std::string, Timeout> timeouts;
  virtual ~Component() = default;
  virtual void setup() {}
  void set_interval(const std::string &name, uint32_t, std::function<void()> cb) { intervals[name] = cb; }
  void cancel_interval(const std::string &name) { intervals.erase(name); }
  void run_interval(const std::string &name) { auto cb = intervals.at(name); cb(); }
  void set_timeout(const std::string &name, uint32_t delay_ms, std::function<void()> cb) {
    timeouts[name] = {fake::now + delay_ms, cb};
  }
  void cancel_timeout(const std::string &name) { timeouts.erase(name); }
  void run_timeouts() {
    while (true) {
      auto it = std::find_if(timeouts.begin(), timeouts.end(), [](const auto &item) {
        return static_cast<int32_t>(fake::now - item.second.due) >= 0;
      });
      if (it == timeouts.end()) return;
      auto cb = it->second.callback; timeouts.erase(it); cb();
    }
  }
};
class GPIOPin { public: void setup() {} void digital_write(bool) {} };
template<class> class CallbackManager;
template<class... Args> class CallbackManager<void(Args...)> {
  std::vector<std::function<void(Args...)>> callbacks_;
public:
  void add(std::function<void(Args...)> &&cb) { callbacks_.push_back(std::move(cb)); }
  void call(Args... args) { for (auto &cb : callbacks_) cb(args...); }
};
namespace switch_ {
class Switch { public: bool state = false; void publish_state(bool value) { state = value; }
protected: virtual void write_state(bool) = 0; };
}
namespace uart {
enum UARTParityOptions { UART_CONFIG_PARITY_NONE, UART_CONFIG_PARITY_EVEN, UART_CONFIG_PARITY_ODD };
enum class UARTFlushResult { UART_FLUSH_RESULT_SUCCESS, UART_FLUSH_RESULT_ASSUMED_SUCCESS,
                             UART_FLUSH_RESULT_TIMEOUT, UART_FLUSH_RESULT_FAILED };
class UARTComponent {
public:
  std::deque<uint8_t> rx;
  std::vector<std::vector<uint8_t>> tx;
  std::vector<uint32_t> tx_times;
  uint32_t baud_rate = 9600;
  uint8_t data_bits = 8, stop_bits = 1;
  UARTParityOptions parity = UART_CONFIG_PARITY_NONE;
  size_t read_byte_calls = 0;
  std::vector<size_t> read_array_sizes;
  size_t fail_read_array_call = 0, partial_read_bytes = 0;
  std::function<void()> after_read_array;
  uint32_t get_baud_rate() const { return baud_rate; }
  uint8_t get_data_bits() const { return data_bits; }
  uint8_t get_stop_bits() const { return stop_bits; }
  UARTParityOptions get_parity() const { return parity; }
  size_t available() const { return rx.size(); }
  bool read_byte(uint8_t *b) { ++read_byte_calls; if (rx.empty()) return false; *b = rx.front(); rx.pop_front(); return true; }
  bool read_array(uint8_t *data, size_t len) {
    read_array_sizes.push_back(len);
    if (rx.size() < len) return false;
    const bool fail = read_array_sizes.size() == fail_read_array_call;
    const size_t count = fail ? std::min(len, partial_read_bytes) : len;
    for (size_t i = 0; i < count; ++i) { data[i] = rx.front(); rx.pop_front(); }
    auto callback = std::move(after_read_array);
    after_read_array = nullptr;
    if (callback) callback();
    return !fail;
  }
  void write_array(const std::vector<uint8_t> &data) { tx.push_back(data); tx_times.push_back(micros()); }
  UARTFlushResult flush() { return UARTFlushResult::UART_FLUSH_RESULT_SUCCESS; }
};
}
namespace network {
struct IPAddress {
  uint32_t value = 0;
  bool v4 = true;
  bool is_ip4() const { return v4; }
  bool is_set() const { return value != 0; }
  char *str_to(char *buf) const { snprintf(buf, 40, "%u.%u.%u.%u", value>>24, (value>>16)&255, (value>>8)&255, value&255); return buf; }
};
inline std::array<IPAddress,5> addresses{};
inline bool connected = false;
constexpr size_t IP_ADDRESS_BUFFER_SIZE = 40;
inline bool is_connected() { return connected; }
inline auto get_ip_addresses() { return addresses; }
}
}

#ifdef USE_ESP8266
struct IPAddress {
  uint32_t value;
  uint8_t operator[](unsigned i) const { return (value >> (24-8*i)) & 255; }
  std::string toString() const { return "192.168.1.100"; }
};
class WiFiClient {
public:
  int fd = -1;
  WiFiClient() = default;
  explicit WiFiClient(int value) : fd(value) {}
  operator bool() const { return fd >= 0; }
  bool connected() { return fd >= 0 && (fake::connections[fd].connected || !fake::connections[fd].rx.empty()); }
  int available() { return fd < 0 ? 0 : fake::connections[fd].rx.size(); }
  int read(uint8_t *data, size_t size) { return (fake::recv)(fd, data, size, 0); }
  size_t write(const uint8_t *data, size_t size) { int n = (fake::send)(fd, data, size, 0); return n > 0 ? n : 0; }
  void stop() { if (fd >= 0) (fake::close)(fd); fd = -1; }
  void setNoDelay(bool) {}
  void setTimeout(unsigned) {}
  IPAddress remoteIP() { return {fake::connections[fd].ip}; }
  uint16_t remotePort() { return fake::connections[fd].port; }
};
// Avoid the function-like accept macro in the Arduino method declaration.
#undef accept
class WiFiServer {
public:
  explicit WiFiServer(uint16_t) {}
  void begin() {}
  void stop() {}
  WiFiClient accept() { sockaddr_in peer{}; return WiFiClient(fake::accept(0, reinterpret_cast<sockaddr *>(&peer), nullptr)); }
};
struct FakeWiFi { IPAddress localIP() { return {0xC0A80164}; } };
inline FakeWiFi WiFi;
#endif
