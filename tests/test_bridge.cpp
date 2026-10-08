#include "host_runtime.h"
#include "../components/modbus_bridge/modbus_bridge.cpp"

using namespace esphome;
using namespace esphome::modbus_bridge;

class TestBridge : public ModbusBridgeComponent {
public:
  using ModbusBridgeComponent::check_tcp_sockets_;
  using ModbusBridgeComponent::poll_uart_response_;
  using ModbusBridgeComponent::poll_trusted_host_lookup_;
  size_t queued() const { return pending_requests_.size(); }
  uint16_t active_tid() const { return transaction_id_from_header_(pending_requests_.front().header); }
  bool running() const { return sock_ >= 0; }
  bool resolving() const { return trusted_hosts_resolving_; }
  bool pending(size_t slot = 0) const { return clients_[slot].trust_pending; }
  bool trusted(size_t slot = 0) const { return clients_[slot].trusted; }
  bool active() const { return rtu_request_active_; }
  uint32_t frame_gap_us() const { return rtu_frame_gap_us_; }
  size_t rtu_buffered() const { return pending_requests_.empty() ? 0 : pending_requests_.front().response.size(); }
  size_t buffered(size_t slot = 0) const {
#ifdef USE_ESP32
    return rx_accu_[slot].size();
#else
    return rx_accu8266_[slot].size();
#endif
  }
};

struct Fixture {
  uart::UARTComponent uart;
  TestBridge bridge;
  explicit Fixture(uint32_t baud_rate = 9600, uint8_t stop_bits = 1,
                   uart::UARTParityOptions parity = uart::UART_CONFIG_PARITY_NONE) {
    fake::now = 0;
    fake::sub_ms = 0;
    fake::connections.clear(); fake::accepts.clear(); fake::next_fd = 3;
    fake::dns_requests.clear(); fake::dns_tasks.clear(); fake::dns_cached.clear();
    fake::dns_error = false; fake::dns_queue_full = false; fake::logs.clear();
    network::connected = true;
    network::addresses = {}; network::addresses[0].value = 0xC0A80164;
    uart.baud_rate = baud_rate;
    uart.stop_bits = stop_bits; uart.parity = parity;
    bridge.set_uart_id(&uart); bridge.setup();
    network_tick();
  }
  void network_tick() { bridge.run_interval("tcp_server_and_network_check"); }
  void poll() { bridge.check_tcp_sockets_(); }
  void advance(uint32_t at) { fake::now = at; bridge.run_timeouts(); }
  int connect(uint32_t ip = 0xC0A80120) { int fd = fake::client(ip); poll(); return fd; }
  void incoming(int fd, const std::vector<uint8_t> &data) {
    auto &rx = fake::connections[fd].rx; rx.insert(rx.end(), data.begin(), data.end());
  }
  void receive(const std::vector<uint8_t> &data, uint32_t at) {
    fake::now = at; uart.rx.insert(uart.rx.end(), data.begin(), data.end()); bridge.poll_uart_response_();
  }
  void settle(uint32_t at) {
    fake::now = at; bridge.poll_uart_response_(); fake::now = at + 7; bridge.poll_uart_response_();
  }
  void resolve(uint32_t ip) {
    fake::run_dns_task(); fake::dns_reply(ip); poll();
  }
};

static std::vector<uint8_t> request(uint16_t tid, uint8_t fc = 4) {
  return {uint8_t(tid >> 8), uint8_t(tid), 0, 0, 0, 6, 1, fc, 0, 18, 0, 1};
}

static std::vector<uint8_t> with_crc(std::vector<uint8_t> data) {
  const uint16_t crc = modbus_crc(data.data(), data.size());
  data.push_back(uint8_t(crc)); data.push_back(uint8_t(crc >> 8)); return data;
}

static void test_tcp_buffer() {
  Fixture f; int fd = f.connect();
  for (uint16_t tid = 1; tid <= 10; ++tid) f.incoming(fd, request(tid));
  f.poll();
  assert(f.bridge.queued() == 8 && f.bridge.buffered() == 24);
  assert(fake::connections[fd].rx.empty());
  f.poll();
  assert(f.bridge.queued() == 10 && f.bridge.buffered() == 0);
  assert(f.uart.tx.size() == 1);
}

static void test_tcp_fragments() {
  Fixture f; int fd = f.connect(); auto data = request(1);
  f.incoming(fd, {data.begin(), data.begin() + 5}); f.poll();
  assert(f.bridge.queued() == 0 && f.bridge.buffered() == 5);
  f.poll(); assert(f.bridge.queued() == 0);
  f.incoming(fd, {data.begin() + 5, data.end()}); f.poll();
  assert(f.bridge.queued() == 1 && f.uart.tx.size() == 1);
}

static void test_echo_and_delayed_response() {
  Fixture f; int fd = f.connect();
  f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
  const auto echo = f.uart.tx.front();
  f.receive(echo, 7); f.settle(14);
  assert(f.uart.tx.size() == 1 && f.bridge.active_tid() == 1);
  f.receive({1,4,2,0,0xA7,0xF8,0x8A}, 50);
  assert(f.uart.tx.size() == 1 && f.bridge.active_tid() == 2);
  const auto &reply = fake::connections[fd].tx;
  assert((reply == std::vector<uint8_t>{0,1,0,0,0,5,1,4,2,0,0xA7}));
  f.advance(55); assert(f.uart.tx.size() == 2);
}

static void test_invalid_responses_keep_deadline() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
  f.receive(with_crc({2,4,2,0,0}), 7); f.settle(14); // other UID
  assert(f.uart.tx.size() == 1);
  f.receive({1,4,2,0,0,0,0}, 100); f.settle(107); // bad CRC
  assert(f.uart.tx.size() == 1);
  f.receive({0x55}, 900); f.settle(907); // noise
  assert(f.uart.tx.size() == 1);
  fake::now = 1001; f.bridge.poll_uart_response_();
  assert(f.uart.tx.size() == 2 && f.bridge.active_tid() == 2);
  assert(std::any_of(fake::logs.begin(), fake::logs.end(), [](const auto &s) {
    return s.find("no valid matching response") != std::string::npos;
  }));
}

static void test_valid_write_and_exception() {
  for (uint8_t fc : {uint8_t(6), uint8_t(4)}) {
    Fixture f; int fd = f.connect(); f.incoming(fd, request(1, fc)); f.poll();
    f.receive(fc == 6 ? f.uart.tx.front() : with_crc({1,0x84,2}), 7);
    assert(f.bridge.queued() == 0 && !fake::connections[fd].tx.empty());
  }
}

static void test_no_response_and_incomplete() {
  for (bool incomplete : {false, true}) {
    Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
    if (incomplete) f.receive({1,4,2,0}, 7);
    fake::now = 1001; f.bridge.poll_uart_response_();
    assert(f.bridge.queued() == 0 && fake::connections[fd].tx.empty());
  }
}

static void test_network_recovery() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  network::addresses = {}; f.network_tick();
  assert(!f.bridge.running() && f.bridge.queued() == 0);
  assert(!fake::connections[fd].connected);
  network::addresses[0] = {1, false}; f.network_tick(); assert(!f.bridge.running());
  network::addresses[0] = {0xC0A80164, true}; network::connected = false;
  f.network_tick(); assert(!f.bridge.running());
  network::connected = true; f.network_tick(); assert(f.bridge.running());
  int next = f.connect(); f.incoming(next, request(2)); f.poll(); assert(f.bridge.active_tid() == 2);
  f.advance(5); assert(f.bridge.active());
}

static void test_dns_pending_and_existing_client() {
  Fixture f;
  f.bridge.add_trusted_network(0xC0A80100, 0xFFFFFF00);
  f.bridge.add_trusted_host("home.example"); f.bridge.set_reject_untrusted_clients(true);
  int local = f.connect(); assert(!f.bridge.resolving());
  int remote = f.connect(0xCB007109); assert(f.bridge.pending(1));
  f.incoming(remote, request(2)); f.incoming(local, request(1)); f.poll();
  assert(f.uart.tx.size() == 1 && f.bridge.active_tid() == 1);
  f.receive(with_crc({1,4,2,0,42}), 20); assert(f.bridge.queued() == 0);
  f.resolve(0xCB007109);
  assert(!f.bridge.pending(1) && f.bridge.trusted(1));
  f.poll(); assert(f.bridge.active_tid() == 2);
}

static void test_dns_failure_and_cache() {
  Fixture f; f.bridge.add_trusted_host("home.example"); f.bridge.set_reject_untrusted_clients(true);
  int remote = f.connect(0xCB007109); f.incoming(remote, request(1));
  f.resolve(0);
  assert(!fake::connections[remote].connected && f.uart.tx.empty());
  int again = f.connect(0xCB007109);
  assert(!fake::connections[again].connected && !f.bridge.resolving());
  fake::now = 60001; remote = f.connect(0xCB007109); assert(f.bridge.resolving());
  f.resolve(0xCB007109); assert(f.bridge.trusted());
}

static void test_dns_shutdown_and_reuse() {
  Fixture f; f.bridge.add_trusted_host("home.example"); f.bridge.set_reject_untrusted_clients(true);
  int old = f.connect(0xCB007109); fake::run_dns_task();
  network::connected = false; f.network_tick(); assert(!fake::connections[old].connected);
  network::connected = true; f.network_tick(); int next = f.connect(0xCB007109);
  f.incoming(next, request(1)); f.resolve(0xCB007109); // obsolete result
  assert(f.bridge.pending() && f.uart.tx.empty());
  f.resolve(0); // current lookup failed; obsolete success must not grant access
  assert(!fake::connections[next].connected && f.uart.tx.empty());
}

static void test_dns_multiple_and_immediate() {
  Fixture f; f.bridge.add_trusted_host("first.example"); f.bridge.add_trusted_host("second.example");
  fake::dns_cached["first.example"] = 0xCB007108;
  int remote = f.connect(0xCB007109); fake::run_dns_task(); f.poll();
  assert(f.bridge.pending());
  f.resolve(0xCB007109);
  assert(!f.bridge.pending() && f.bridge.trusted());
  f.incoming(remote, request(1)); f.poll(); assert(f.uart.tx.size() == 1);
}

static void test_echo_before_exception() {
  for (bool swapped : {false, true}) {
    Fixture f; f.bridge.set_crc_bytes_swapped(swapped);
    int fd = f.connect(); auto req = request(1);
    req[8] = 0x90; req[9] = 0; req[11] = 72;
    f.incoming(fd, req); f.poll();
    auto capture = f.uart.tx.front();
    auto exception = with_crc({1,0x84,2});
    if (swapped) std::swap(exception[3], exception[4]);
    capture.insert(capture.end(), exception.begin(), exception.end());
    f.receive(capture, 10); f.settle(17);
    assert(f.bridge.queued() == 0);
    assert((fake::connections[fd].tx == std::vector<uint8_t>{0,1,0,0,0,3,1,0x84,2}));
  }
}

static void test_stale_write_before_matching_reply() {
  for (uint8_t fc : {uint8_t(5), uint8_t(6), uint8_t(15), uint8_t(16)}) {
    for (bool wrong_address : {false, true}) {
      Fixture f; int fd = f.connect(); auto req = request(1,fc);
      if (fc == 5) { req[10] = 0xFF; req[11] = 0; }
      if (fc == 15) { req[5] = 8; req.insert(req.end(), {1,1}); }
      if (fc == 16) { req[5] = 9; req.insert(req.end(), {2,0,1}); }
      f.incoming(fd, req); f.poll();
      auto current = with_crc({1,fc,req[8],req[9],req[10],req[11]});
      auto stale = std::vector<uint8_t>(current.begin(), current.end() - 2);
      ++stale[wrong_address ? 3 : 5];
      auto capture = with_crc(stale);
      capture.insert(capture.end(), current.begin(), current.end());
      f.receive(capture, 10); f.settle(17);
      assert(f.bridge.queued() == 0);
      auto expected = std::vector<uint8_t>{0,1,0,0,0,6};
      expected.insert(expected.end(), current.begin(), current.end() - 2);
      assert(fake::connections[fd].tx == expected);
    }
  }
}

static void test_partial_noisy_response() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  const auto reply = with_crc({1,4,2,0,42});
  std::vector<uint8_t> capture{0x55,0xAA};
  capture.insert(capture.end(), reply.begin(), reply.begin() + 4);
  f.receive(capture, 10); f.settle(17);
  assert(f.bridge.queued() == 1 && fake::connections[fd].tx.empty());
  f.receive({reply.begin() + 4, reply.end()}, 50); f.settle(57);
  assert(f.bridge.queued() == 0 && fake::connections[fd].tx.size() == 11);
}

static void test_delayed_short_response_prefix() {
  for (uint32_t baud : {9600U,19200U,115200U}) {
    for (uint8_t fc : {uint8_t(1), uint8_t(2), uint8_t(3), uint8_t(4),
                       uint8_t(5), uint8_t(6), uint8_t(0x0F), uint8_t(0x10)}) {
      for (size_t prefix : {size_t(1), size_t(2)}) {
        for (bool noisy : {false, true}) {
          Fixture f(baud); int fd = f.connect(); auto req = request(1,fc);
          if (fc == 5) { req[10] = 0xFF; req[11] = 0; }
          if (fc == 0x0F) { req[5] = 8; req.insert(req.end(), {1,1}); }
          if (fc == 0x10) { req[5] = 9; req.insert(req.end(), {2,0,42}); }
          f.incoming(fd, req); f.poll();
          const auto reply = fc <= 2 ? with_crc({1,fc,1,1})
                             : fc <= 4 ? with_crc({1,fc,2,0,42})
                             : fc <= 6 ? f.uart.tx.front()
                             : with_crc({1,fc,0,18,0,1});
          std::vector<uint8_t> capture;
          if (noisy) capture = {0x55,0xAA};
          capture.insert(capture.end(), reply.begin(), reply.begin() + prefix);
          const auto drops = f.bridge.get_drops_rtu_incomplete();
          f.receive(capture, 10); f.settle(20); f.settle(34);
          assert(f.bridge.active() && f.bridge.rtu_buffered() == capture.size());
          assert(fake::connections[fd].tx.empty() && f.bridge.get_drops_rtu_incomplete() == drops);
          f.receive({reply.begin() + prefix, reply.end()}, 50);
          if (!noisy) assert(f.bridge.queued() == 0); // Clean frames still complete immediately.
          f.settle(60);
          std::vector<uint8_t> expected{0,1,0,0,0,uint8_t(reply.size() - 2)};
          expected.insert(expected.end(), reply.begin(), reply.end() - 2);
          assert(f.bridge.queued() == 0 && fake::connections[fd].tx == expected);
        }
      }
    }
  }
}

static void test_delayed_exception_prefix() {
  for (uint8_t fc : {uint8_t(3), uint8_t(0x10)}) {
    for (size_t prefix : {size_t(1), size_t(2)}) {
      Fixture f; int fd = f.connect(); f.incoming(fd, request(1,fc)); f.poll();
      const auto reply = with_crc({1,uint8_t(fc | 0x80),2});
      f.receive({reply.begin(), reply.begin() + prefix}, 10); f.settle(20);
      assert(f.bridge.active() && f.bridge.rtu_buffered() == prefix);
      f.receive({reply.begin() + prefix, reply.end()}, 50);
      assert(f.bridge.queued() == 0);
      assert((fake::connections[fd].tx == std::vector<uint8_t>{0,1,0,0,0,3,1,uint8_t(fc | 0x80),2}));
    }
  }
}

static void test_single_uid_after_echo_and_noise() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  auto capture = f.uart.tx.front(); capture.insert(capture.end(), {0x55,0xAA,1});
  f.receive(capture, 10); f.settle(20);
  assert(f.bridge.active() && f.bridge.rtu_buffered() == capture.size());
  const auto reply = with_crc({1,4,2,0,42});
  f.receive({reply.begin() + 1, reply.end()}, 50); f.settle(60);
  assert(f.bridge.queued() == 0);
  assert((fake::connections[fd].tx == std::vector<uint8_t>{0,1,0,0,0,5,1,4,2,0,42}));
}

static void test_short_prefix_does_not_hide_valid_frame() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  f.receive({1}, 10); f.settle(20);
  const auto reply = with_crc({1,4,2,0,42});
  auto capture = reply; capture.back() ^= 1;
  capture.insert(capture.end(), reply.begin(), reply.end()); capture.push_back(1);
  const auto drops = f.bridge.get_drops_rtu_crc();
  f.receive(capture, 50); f.settle(60);
  assert(f.bridge.queued() == 0 && f.bridge.get_drops_rtu_crc() == drops);
  assert((fake::connections[fd].tx == std::vector<uint8_t>{0,1,0,0,0,5,1,4,2,0,42}));
}

static void test_short_prefix_keeps_original_timeout_and_resets() {
  for (bool noisy : {false, true}) {
    Fixture f; int fd = f.connect();
    f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
    const auto timeouts = f.bridge.get_timeouts();
    const auto drops = f.bridge.get_drops_rtu_incomplete();
    const std::vector<uint8_t> capture = noisy ? std::vector<uint8_t>{0x55,1} : std::vector<uint8_t>{1};
    f.receive(capture, 900); f.settle(920);
    assert(f.bridge.rtu_buffered() == capture.size());
    f.advance(1000); f.bridge.poll_uart_response_();
    assert(f.bridge.active_tid() == 1 && f.bridge.get_timeouts() == timeouts);
    f.advance(1001); f.bridge.poll_uart_response_();
    assert(f.bridge.active_tid() == 2 && f.uart.tx.size() == 2 && f.bridge.rtu_buffered() == 0);
    assert(f.bridge.get_timeouts() == timeouts + 1 && f.bridge.get_drops_rtu_incomplete() == drops + 1);
    f.receive(with_crc({1,4,2,0,42}), 1020);
    assert(f.bridge.queued() == 0);
    assert((fake::connections[fd].tx == std::vector<uint8_t>{0,2,0,0,0,5,1,4,2,0,42}));
  }
}

static void test_unrelated_short_noise_is_discarded() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  const auto drops = f.bridge.get_drops_rtu_incomplete();
  f.receive({0x55}, 10); f.settle(20);
  assert(f.bridge.active() && f.bridge.rtu_buffered() == 0);
  assert(f.bridge.get_drops_rtu_incomplete() == drops + 1);
  f.receive(with_crc({1,4,2,0,42}), 50);
  assert(f.bridge.queued() == 0 && fake::connections[fd].tx.size() == 11);
}

static void test_short_prefix_still_requires_matching_fc_and_crc() {
  for (bool wrong_fc : {false, true}) {
    Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
    f.receive({1}, 10); f.settle(20);
    auto invalid = with_crc({1,uint8_t(wrong_fc ? 3 : 4),2,0,42});
    if (!wrong_fc) invalid.back() ^= 0x80;
    const auto drops = wrong_fc ? f.bridge.get_drops_rtu_mismatch() : f.bridge.get_drops_rtu_crc();
    f.receive({invalid.begin() + 1, invalid.end()}, 50); f.settle(60);
    assert(f.bridge.active() && fake::connections[fd].tx.empty());
    const auto after = wrong_fc ? f.bridge.get_drops_rtu_mismatch() : f.bridge.get_drops_rtu_crc();
    assert(after == drops + 1 && f.bridge.rtu_buffered() == 0);
    f.receive(with_crc({1,4,2,0,42}), 80);
    assert(f.bridge.queued() == 0 && fake::connections[fd].tx.size() == 11);
  }
}

static void test_short_prefix_capture_remains_bounded() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  std::vector<uint8_t> capture(kMaxRtuCapture - 1, 0x55); capture.push_back(1);
  const auto drops = f.bridge.get_drops_rtu_incomplete();
  f.receive(capture, 10); f.settle(20);
  assert(f.bridge.rtu_buffered() == kMaxRtuCapture && f.bridge.get_drops_rtu_incomplete() == drops);
  f.receive({4}, 50);
  assert(f.bridge.active() && f.bridge.rtu_buffered() == 0);
  assert(f.bridge.get_drops_rtu_incomplete() == drops + 1);
  f.receive(with_crc({1,4,2,0,42}), 80);
  assert(f.bridge.queued() == 0 && fake::connections[fd].tx.size() == 11);
}

static void test_bad_crc_candidate_and_trailing_noise() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  const auto reply = with_crc({1,4,2,0,42});
  auto capture = reply; capture.back() ^= 1;
  capture.insert(capture.end(), reply.begin(), reply.end());
  capture.insert(capture.end(), {0x55,0xAA,0xFF});
  f.receive(capture, 10); f.settle(17);
  assert(f.bridge.queued() == 0 && fake::connections[fd].tx.size() == 11);
}

static void test_partial_noise_keeps_original_timeout() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  const auto before = f.bridge.get_timeouts();
  f.receive({0x55,1,4,2,0}, 900); f.settle(907);
  assert(f.bridge.active());
  f.advance(1001); f.bridge.poll_uart_response_();
  assert(f.bridge.queued() == 0 && f.bridge.get_timeouts() == before + 1);
}

static void test_unknown_function_unchanged() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1,0x41)); f.poll();
  auto reply = with_crc({1,0x41,0xCA,0xFE,0,0x42});
  f.receive(reply, 10); f.settle(17);
  assert(f.bridge.queued() == 0);
  assert((fake::connections[fd].tx == std::vector<uint8_t>{0,1,0,0,0,6,1,0x41,0xCA,0xFE,0,0x42}));
}

static void test_unknown_function_crc_ending_in_uid() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1,0x41)); f.poll();
  const auto reply = with_crc({1,0x41,0,19});
  assert(reply.back() == 1);
  f.receive(reply, 10); f.settle(20);
  std::vector<uint8_t> expected{0,1,0,0,0,uint8_t(reply.size() - 2)};
  expected.insert(expected.end(), reply.begin(), reply.end() - 2);
  assert(f.bridge.queued() == 0 && fake::connections[fd].tx == expected);
}

static void test_bus_gap_and_immediate_tcp_reply() {
  for (uint32_t baud : {9600U,19200U,115200U}) {
    Fixture f(baud); int fd = f.connect();
    f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
    f.receive(with_crc({1,4,2,0,42}), 20);
    assert(!fake::connections[fd].tx.empty());
    assert(fake::now == 20 && f.uart.tx.size() == 1 && !f.bridge.active());
    uint32_t wait_ms = (f.bridge.frame_gap_us() + 999) / 1000;
    assert(wait_ms == (baud == 9600 ? 5U : (baud == 19200 ? 3U : 2U)));
    f.advance(20 + wait_ms - 1); assert(f.uart.tx.size() == 1);
    f.advance(20 + wait_ms); assert(f.uart.tx.size() == 2 && f.bridge.active());
    assert(f.uart.tx_times.back() - 20000U >= f.bridge.frame_gap_us());
  }
}

static void test_bus_gap_for_new_request_and_late_bytes() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
  f.receive(with_crc({1,4,2,0,42}), 20);
  f.incoming(fd, request(2)); f.poll(); assert(f.uart.tx.size() == 1);
  f.uart.rx.push_back(0x55);
  f.advance(25); assert(f.uart.tx.size() == 1 && f.uart.rx.empty());
  f.advance(29); assert(f.uart.tx.size() == 1);
  f.advance(30); assert(f.uart.tx.size() == 2);
}

static void test_bus_gap_with_parity_and_two_stop_bits() {
  Fixture f(1200,2,uart::UART_CONFIG_PARITY_EVEN);
  int fd = f.connect(); f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
  f.receive(with_crc({1,4,2,0,42}), 100);
  assert(f.bridge.frame_gap_us() == 35000);
  f.advance(134); assert(f.uart.tx.size() == 1);
  f.advance(135); assert(f.uart.tx.size() == 2);
}

static void test_bus_gap_micros_wrap() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
  const uint32_t at = 4294966;
  f.receive(with_crc({1,4,2,0,42}), at);
  assert(f.uart.tx.size() == 1);
  f.advance(at + 4); assert(f.uart.tx.size() == 1);
  f.advance(at + 5); assert(f.uart.tx.size() == 2);
  assert(f.uart.tx_times.back() - at * 1000U >= f.bridge.frame_gap_us());
}

static void test_disable_cancels_scheduled_tx() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
  f.receive(with_crc({1,4,2,0,42}), 20);
  f.bridge.set_enabled(false); f.advance(30);
  assert(f.bridge.queued() == 0 && f.uart.tx.size() == 1 && f.bridge.timeouts.empty());
}

static void test_disconnect_before_scheduled_tx() {
  Fixture f; int fd = f.connect(); f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
  f.receive(with_crc({1,4,2,0,42}), 20);
  fake::connections[fd].connected = false; f.poll(); f.advance(30);
  assert(f.bridge.queued() == 0 && f.uart.tx.size() == 1);
}

static void test_protection_rechecked_at_dispatch() {
  for (uint8_t fc : {uint8_t(4), uint8_t(6)}) {
    for (bool toggle_during_gap : {false, true}) {
      Fixture f; f.bridge.add_trusted_network(0xC0A80100, 0xFFFFFF00);
      int local = f.connect(); int remote = f.connect(0xCB007109);
      f.incoming(local, request(1)); f.poll();
      f.incoming(remote, request(2,fc)); f.poll();
      auto before = fc == 4 ? f.bridge.get_drop_untrusted_reads() : f.bridge.get_drop_untrusted_writes();
      if (toggle_during_gap) f.receive(with_crc({1,4,2,0,42}), 20);
      if (fc == 4) f.bridge.set_protect_reads_for_untrusted_clients(true);
      else f.bridge.set_protect_writes_for_untrusted_clients(true);
      if (!toggle_during_gap) f.receive(with_crc({1,4,2,0,42}), 20);
      f.advance(25);
      auto after = fc == 4 ? f.bridge.get_drop_untrusted_reads() : f.bridge.get_drop_untrusted_writes();
      assert(f.uart.tx.size() == 1 && f.bridge.queued() == 0 && after == before + 1);
      assert(fake::connections[remote].tx.empty());
    }
  }
}

static void test_protection_without_trust_rules() {
  Fixture f; int fd = f.connect();
  f.incoming(fd, request(1)); f.incoming(fd, request(2,6)); f.poll();
  f.bridge.set_protect_writes_for_untrusted_clients(true);
  f.receive(with_crc({1,4,2,0,42}), 20); f.advance(25);
  assert(f.uart.tx.size() == 2 && f.uart.tx.back()[1] == 6);
}

static void test_active_request_not_cancelled_by_protection() {
  Fixture f; f.bridge.add_trusted_network(0xC0A80100, 0xFFFFFF00);
  int remote = f.connect(0xCB007109); f.incoming(remote, request(1,6)); f.poll();
  f.bridge.set_protect_writes_for_untrusted_clients(true);
  f.receive(f.uart.tx.front(), 20);
  assert(f.bridge.queued() == 0 && fake::connections[remote].tx.size() == 12);
}

static void test_invalid_tcp_lengths_counted_once() {
  for (uint16_t len : {uint16_t(0), uint16_t(1), uint16_t(255), uint16_t(65535)}) {
    Fixture f; int fd = f.connect(); auto req = request(1);
    req[4] = len >> 8; req[5] = len & 255;
    auto before = f.bridge.get_drops_tcp_len();
    f.incoming(fd, {req.begin(), req.begin() + 5}); f.poll();
    assert(f.bridge.get_drops_tcp_len() == before);
    f.incoming(fd, {req.begin() + 5, req.end()}); f.poll(); f.poll();
    assert(f.bridge.get_drops_tcp_len() == before + 1 && f.uart.tx.empty());
    f.incoming(fd, request(2)); f.poll(); assert(f.uart.tx.size() == 1);
  }
}

static void test_trusted_priority_during_bus_gap() {
  Fixture f; f.bridge.add_trusted_network(0xC0A80100, 0xFFFFFF00);
  f.bridge.set_protect_writes_for_untrusted_clients(true);
  int local = f.connect(); int remote = f.connect(0xCB007109);
  f.incoming(local, request(1)); f.poll();
  f.incoming(remote, request(2)); f.poll();
  f.receive(with_crc({1,4,2,0,42}), 20);
  assert(!f.bridge.active() && f.bridge.active_tid() == 2);
  f.incoming(local, request(3)); f.poll();
  f.advance(25);
  assert(f.bridge.active() && f.bridge.active_tid() == 3 && f.uart.tx.size() == 2);
}

static void test_disconnect_preserves_inflight_request() {
  Fixture f; int first = f.connect(); int second = f.connect(0xC0A80121);
  f.incoming(first, request(1)); f.poll(); f.incoming(second, request(2)); f.poll();
  fake::connections[first].connected = false; f.poll();
  assert(f.bridge.active() && f.bridge.active_tid() == 1);
  f.receive(with_crc({1,4,2,0,42}), 20);
  assert(fake::connections[second].tx.empty() && f.uart.tx.size() == 1);
  f.advance(25); assert(f.bridge.active_tid() == 2 && f.uart.tx.size() == 2);
}

static void test_automation_disables_bridge() {
  for (bool on_send : {true, false}) {
    Fixture f; int fd = f.connect();
    auto disable = [&f](int, int) { f.bridge.set_enabled(false); };
    if (on_send) f.bridge.add_on_rtu_send_callback(disable);
    else f.bridge.add_on_rtu_receive_callback(disable);
    f.incoming(fd, request(1)); f.incoming(fd, request(2)); f.poll();
    if (!on_send) f.receive(with_crc({1,4,2,0,42}), 20);
    f.advance(30);
    assert(!f.bridge.is_enabled() && !f.bridge.active() && f.bridge.queued() == 0 && f.uart.tx.size() == 1);
  }
}

static void test_client_count_automation_disables_bridge() {
  for (int event = 0; event < 5; ++event) {
    Fixture f;
    if (event == 3) {
      f.bridge.add_trusted_host("home.example"); f.bridge.set_reject_untrusted_clients(true);
    }
    int fd = event == 0 ? -1 : f.connect();
    bool fired = false;
    f.bridge.add_on_tcp_clients_changed_callback([&f, &fired, event](int count) {
      if ((event == 0 && count > 0) || (event != 0 && count == 0)) {
        fired = true; f.bridge.set_enabled(false);
      }
    });
    if (event == 0) f.connect();
    else if (event == 1) { fake::now = 60001; f.poll(); }
    else if (event == 2) { fake::connections[fd].connected = false; f.poll(); }
    else if (event == 3) f.resolve(0);
    else {
      // Only the slot removed by lowering the limit remains connected.
      int second = f.connect(0xC0A80121);
      fake::connections[fd].connected = false;
      f.bridge.set_tcp_allowed_clients(1); f.poll();
      assert(!fake::connections[second].connected);
    }
    assert(fired && !f.bridge.is_enabled() && !f.bridge.running() && f.bridge.queued() == 0);
  }
}

static void test_client_count_automation_reenables_bridge() {
  Fixture f; int fd = f.connect();
  f.bridge.add_on_tcp_clients_changed_callback([&f](int count) {
    if (count == 0) { f.bridge.set_enabled(false); f.bridge.set_enabled(true); }
  });
  fake::now = 60001; f.poll();
  assert(f.bridge.is_enabled() && !f.bridge.running() && !fake::connections[fd].connected);
  f.network_tick(); fd = f.connect();
  f.incoming(fd, request(1)); f.poll(); assert(f.bridge.active());
}

static void test_dns_pending_disconnect_frees_slots() {
  Fixture f;
  f.bridge.add_trusted_network(0xC0A80100, 0xFFFFFF00);
  f.bridge.add_trusted_host("home.example"); f.bridge.set_reject_untrusted_clients(true);
  int first = f.connect(0xCB007109), second = f.connect(0xCB00710A);
  assert(f.bridge.pending(0) && f.bridge.pending(1));
  fake::connections[first].connected = false; fake::connections[second].connected = false;
  int local = f.connect();
  assert(fake::connections[local].connected && f.bridge.resolving());
  f.incoming(local, request(1)); f.poll();
  assert(f.bridge.active_tid() == 1 && f.uart.tx.size() == 1);
}

static void test_dns_pending_buffer_preserves_requests() {
  Fixture f; f.bridge.add_trusted_host("home.example");
  f.bridge.set_reject_untrusted_clients(true);
  int remote = f.connect(0xCB007109); const auto req = request(1);
  f.incoming(remote, req);
  for (int i = 0; i < 4; ++i) f.poll();
  assert(fake::connections[remote].rx.empty() && f.bridge.buffered() == req.size() && f.uart.tx.empty());
  f.resolve(0xCB007109); f.poll();
  assert(fake::connections[remote].rx.empty() && f.bridge.active_tid() == 1);
}

static void test_dns_pending_disconnect_after_data() {
  Fixture f;
  f.bridge.add_trusted_network(0xC0A80100, 0xFFFFFF00);
  f.bridge.add_trusted_host("home.example"); f.bridge.set_reject_untrusted_clients(true);
  int first = f.connect(0xCB007109), second = f.connect(0xCB00710A);
  f.incoming(first, request(1)); f.incoming(second, request(2));
  fake::connections[first].connected = false; fake::connections[second].connected = false;
  f.poll(); // Drain bytes preceding EOF, but never dispatch them before authorization.
  int local = f.connect();
  assert(fake::connections[local].connected && f.bridge.resolving() && f.uart.tx.empty());
  f.incoming(local, request(3)); f.poll(); assert(f.bridge.active_tid() == 3);
}

static void test_dns_pending_buffer_is_bounded() {
  Fixture f; f.bridge.add_trusted_host("home.example"); f.bridge.set_reject_untrusted_clients(true);
  int remote = f.connect(0xCB007109);
  for (int i = 0; i < 50; ++i) f.incoming(remote, request(i));
  for (int i = 0; i < 3; ++i) f.poll();
  assert(!fake::connections[remote].connected && f.uart.tx.empty() && f.bridge.queued() == 0);
}

static void test_runtime_client_limit_changes() {
  Fixture f; f.bridge.set_tcp_allowed_clients(8);
  std::vector<int> clients;
  for (int i = 0; i < 8; ++i) {
    clients.push_back(f.connect(0xC0A80120 + i));
    assert(fake::connections[clients.back()].connected);
  }
  int excess = f.connect(0xC0A80140); assert(!fake::connections[excess].connected);
  f.incoming(clients[6], request(1)); f.poll();
  f.incoming(clients[7], request(2)); f.poll();
  f.incoming(clients[0], request(3)); f.poll(); assert(f.bridge.queued() == 3);
  f.bridge.set_tcp_allowed_clients(2); f.poll();
  for (int i = 2; i < 8; ++i) assert(!fake::connections[clients[i]].connected);
  assert(f.bridge.queued() == 2 && f.bridge.active_tid() == 1);
  f.receive(with_crc({1,4,2,0,42}), 20); f.advance(25);
  assert(fake::connections[clients[6]].tx.empty() && f.bridge.active_tid() == 3);
  f.receive(with_crc({1,4,2,0,43}), 40);
  assert(f.bridge.queued() == 0 && fake::connections[clients[0]].tx[1] == 3);
  f.bridge.set_tcp_allowed_clients(8);
  int replacement = f.connect(0xC0A80141);
  assert(fake::connections[replacement].connected && f.bridge.queued() == 0);
}

static void test_client_limit_changed_by_automation() {
  for (bool on_send : {false, true}) {
    Fixture f;
    if (on_send) f.bridge.add_on_rtu_send_callback([&f](int, int) { f.bridge.set_tcp_allowed_clients(8); });
    else f.bridge.add_on_tcp_clients_changed_callback([&f](int count) {
      if (count == 1) f.bridge.set_tcp_allowed_clients(8);
    });
    int fd = f.connect(); f.incoming(fd, request(1)); f.poll();
    assert(f.bridge.active());
    f.poll();
    for (int i = 1; i < 8; ++i) {
      int next = f.connect(0xC0A80120 + i); assert(fake::connections[next].connected);
    }
  }
}

int main() {
  test_tcp_buffer(); test_tcp_fragments(); test_echo_and_delayed_response();
  test_invalid_responses_keep_deadline(); test_valid_write_and_exception(); test_no_response_and_incomplete();
  test_network_recovery(); test_dns_pending_and_existing_client(); test_dns_failure_and_cache();
  test_dns_shutdown_and_reuse(); test_dns_multiple_and_immediate();
  test_echo_before_exception(); test_stale_write_before_matching_reply(); test_partial_noisy_response();
  test_delayed_short_response_prefix(); test_delayed_exception_prefix(); test_single_uid_after_echo_and_noise();
  test_short_prefix_does_not_hide_valid_frame(); test_short_prefix_keeps_original_timeout_and_resets();
  test_unrelated_short_noise_is_discarded(); test_unknown_function_crc_ending_in_uid();
  test_short_prefix_still_requires_matching_fc_and_crc(); test_short_prefix_capture_remains_bounded();
  test_bad_crc_candidate_and_trailing_noise(); test_partial_noise_keeps_original_timeout(); test_unknown_function_unchanged();
  test_bus_gap_and_immediate_tcp_reply(); test_bus_gap_for_new_request_and_late_bytes(); test_bus_gap_micros_wrap();
  test_bus_gap_with_parity_and_two_stop_bits();
  test_disable_cancels_scheduled_tx(); test_disconnect_before_scheduled_tx(); test_protection_rechecked_at_dispatch();
  test_protection_without_trust_rules(); test_active_request_not_cancelled_by_protection(); test_invalid_tcp_lengths_counted_once();
  test_trusted_priority_during_bus_gap(); test_disconnect_preserves_inflight_request(); test_automation_disables_bridge();
  test_client_count_automation_disables_bridge(); test_client_count_automation_reenables_bridge();
  test_dns_pending_disconnect_frees_slots(); test_dns_pending_buffer_preserves_requests();
  test_dns_pending_disconnect_after_data(); test_dns_pending_buffer_is_bounded();
  test_runtime_client_limit_changes(); test_client_limit_changed_by_automation();
#ifdef USE_ESP32
  puts("ESP32: 47 bridge regression tests passed");
#else
  puts("ESP8266: 47 bridge regression tests passed");
#endif
}
