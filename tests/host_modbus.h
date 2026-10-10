#pragma once

#include <memory>
#include <span>

// Only the hub boundary is simulated. Callback/teardown ordering follows the
// ModbusClientDevice contract; the complete bridge runs against it.
namespace esphome::modbus {
enum class ExceptionCode : uint8_t { ILLEGAL_DATA_ADDRESS = 2 };
struct CommandOptions {
  bool continuous = false;
  bool allow_broadcast_read = false;
  bool expect_broadcast_write_response = false;
};
class ModbusClientHub;
class ModbusClientDevice {
public:
  virtual ~ModbusClientDevice();
  void set_parent(ModbusClientHub *parent) { parent_ = parent; }
  void clear_tx_queue_for_device();
  virtual void on_sent(std::span<const uint8_t>) {}
  virtual void on_response(std::span<const uint8_t>, std::span<const uint8_t>) {}
  virtual void on_error(std::span<const uint8_t>, ExceptionCode) {}
  virtual bool on_no_response(std::span<const uint8_t>) { return false; }
  virtual void on_not_sent(std::span<const uint8_t>) {}
protected:
  ModbusClientHub *parent_{nullptr};
};
class ModbusClientHub {
public:
  struct Entry {
    uint8_t address;
    std::vector<uint8_t> pdu;
    ModbusClientDevice *device;
    CommandOptions options;
    bool sent = false;
  };
  std::deque<std::unique_ptr<Entry>> entries;
  bool delivering = false, refuse = false;
  bool queue_pdu(uint8_t address, std::span<const uint8_t> pdu, ModbusClientDevice *device, CommandOptions options = {}) {
    // The bridge must defer new submissions until the completed entry is swept,
    // rather than relying on native duplicate coalescing inside a callback.
    assert(!delivering);
    if (refuse || pdu.empty() || pdu.size() > 253 || (pdu[0] & 0x80)) return false;
    entries.push_back(std::make_unique<Entry>(Entry{address, {pdu.begin(), pdu.end()}, device, options}));
    return true;
  }
  void clear_tx_queue_for_device(ModbusClientDevice *device) {
    for (auto it = entries.begin(); it != entries.end();) {
      if ((*it)->device != device) { ++it; continue; }
      if ((*it)->sent) { (*it)->device = nullptr; ++it; }
      else it = entries.erase(it);
    }
  }
  void transmit() {
    assert(!entries.empty() && !entries.front()->sent);
    auto &entry = *entries.front(); entry.sent = true;
    delivering = true;
    if (entry.device) entry.device->on_sent(entry.pdu);
    delivering = false;
  }
  enum Outcome { RESPONSE, ERROR, TIMEOUT, NOT_SENT };
  void complete(Outcome outcome, const std::vector<uint8_t> &response = {}) {
    assert(!entries.empty());
    auto entry = std::move(entries.front()); entries.pop_front();
    assert(outcome == NOT_SENT || entry->sent);
    delivering = true;
    if (entry->device) {
      switch (outcome) {
      case RESPONSE: entry->device->on_response(entry->pdu, response); break;
      case ERROR: entry->device->on_error(entry->pdu, ExceptionCode::ILLEGAL_DATA_ADDRESS); break;
      case TIMEOUT: assert(!entry->device->on_no_response(entry->pdu)); break;
      case NOT_SENT: entry->device->on_not_sent(entry->pdu); break;
      }
    }
    delivering = false;
  }
};
inline void ModbusClientDevice::clear_tx_queue_for_device() { parent_->clear_tx_queue_for_device(this); }
inline ModbusClientDevice::~ModbusClientDevice() { if (parent_) clear_tx_queue_for_device(); }
}
