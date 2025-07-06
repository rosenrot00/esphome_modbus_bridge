// modbus_bridge.cpp – optimiert für lange Modbus-Antworten (z.B. viele Register)
#include "modbus_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <lwip/sockets.h>
#include <fcntl.h>

namespace esphome {
namespace modbus_bridge {

static const char *const TAG = "modbus_bridge";

ModbusBridgeComponent::ModbusBridgeComponent() {}

void ModbusBridgeComponent::setup() {
  this->set_timeout("modbus_bridge_setup", 5000, [this]() {
    this->initialize_tcp_server_();
  });
}

void ModbusBridgeComponent::initialize_tcp_server_() {
  this->sock_ = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (this->sock_ < 0) {
    ESP_LOGE(TAG, "Socket creation failed");
    return;
  }

  fcntl(this->sock_, F_SETFL, O_NONBLOCK);

  struct sockaddr_in server_addr = {};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(this->tcp_port_);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  bind(this->sock_, (struct sockaddr *)&server_addr, sizeof(server_addr));
  listen(this->sock_, 4);

  ESP_LOGI(TAG, "Listening on port %d", this->tcp_port_);

  for (auto &c : this->clients_) c.fd = -1;
  pending_request_.active = false;
}

void ModbusBridgeComponent::loop() {
  if (this->sock_ < 0 || pending_request_.active) return;

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(this->sock_, &read_fds);
  int maxfd = this->sock_;

  for (auto &c : this->clients_) {
    if (c.fd >= 0) {
      FD_SET(c.fd, &read_fds);
      if (c.fd > maxfd) maxfd = c.fd;
    }
  }

  struct timeval timeout = {0, 0};
  int sel = select(maxfd + 1, &read_fds, NULL, NULL, &timeout);
  if (sel < 0) return;

  if (FD_ISSET(this->sock_, &read_fds)) {
    int newfd = accept(this->sock_, NULL, NULL);
    if (newfd >= 0) {
      fcntl(newfd, F_SETFL, O_NONBLOCK);
      for (auto &c : this->clients_) {
        if (c.fd < 0) {
          c.fd = newfd;
          ESP_LOGD(TAG, "Client connected: %d", newfd);
          break;
        }
      }
    }
  }

  for (auto &c : this->clients_) {
    if (c.fd >= 0 && FD_ISSET(c.fd, &read_fds)) {
      uint8_t header[7];
      int r = recv(c.fd, header, 7, 0);
      if (r <= 0) {
        close(c.fd);
        c.fd = -1;
        continue;
      }
      if (r < 7) continue;

      uint16_t len = (header[4] << 8) | header[5];
      uint8_t uid = header[6];
      std::vector<uint8_t> pdu(len - 1);
      if (recv(c.fd, pdu.data(), len - 1, 0) < 0) continue;

      std::vector<uint8_t> rtu;
      rtu.push_back(uid);
      rtu.insert(rtu.end(), pdu.begin(), pdu.end());
      append_crc(rtu);

      if (debug_) {
        ESP_LOGD(TAG, "TCP->RTU UID: %d, FC: 0x%02X, LEN: %d", uid, pdu[0], len);
        char buf[rtu.size() * 3 + 1];
        char *ptr = buf;
        for (auto b : rtu) ptr += sprintf(ptr, "%02X ", b);
        *ptr = '\0';
        ESP_LOGD(TAG, "RTU send: %s", buf);
      }

      this->uart_->write_array(rtu);
      this->uart_->flush();

      pending_request_.client_fd = c.fd;
      memcpy(pending_request_.header, header, 7);
      pending_request_.response.clear();
      pending_request_.active = true;
      pending_request_.start_time = millis();
      this->set_timeout("modbus_rx_poll", 10, [this]() { this->poll_uart_response_(); });
      break;
    }
  }
}

void ModbusBridgeComponent::poll_uart_response_() {
  if (!pending_request_.active) return;

  while (this->uart_->available()) {
    uint8_t b;
    this->uart_->read_byte(&b);
    pending_request_.response.push_back(b);
  }

  std::vector<uint8_t> &response = pending_request_.response;

  bool has_min_header = response.size() >= 5;
  bool has_full_data = false;

  if (has_min_header) {
    uint8_t byte_count = response[2];  // data length in bytes
    size_t expected = 3 + byte_count + 2;  // 3 header + data + 2 CRC
    if (response.size() >= expected) {
      has_full_data = true;
    }
  }

  if (has_full_data) {
    if (debug_) {
      char buf[response.size() * 3 + 1];
      char *ptr = buf;
      for (auto b : response) ptr += sprintf(ptr, "%02X ", b);
      *ptr = '\0';
      ESP_LOGD(TAG, "RTU recv (%d bytes): %s", (int)response.size(), buf);

      // Optional: Register dump for FC 0x03
      if (response[1] == 0x03) {
        uint8_t bc = response[2];
        std::string reg_dump;
        for (int i = 0; i < bc; i += 2) {
          if (3 + i + 1 < response.size()) {
            uint16_t val = (response[3 + i] << 8) | response[3 + i + 1];
            char rbuf[8];
            sprintf(rbuf, "%04X ", val);
            reg_dump += rbuf;
          }
        }
        ESP_LOGD(TAG, "Register values (hex): %s", reg_dump.c_str());
      }
    }

    std::vector<uint8_t> tcp;
    uint16_t pdulen = response.size() - 3;
    tcp.insert(tcp.end(), pending_request_.header, pending_request_.header + 4);
    tcp.push_back(0);
    tcp.push_back(pdulen + 1);
    tcp.push_back(response[0]);
    tcp.insert(tcp.end(), response.begin() + 1, response.end() - 2);

    if (debug_) {
      char tbuf[tcp.size() * 3 + 1];
      char *tptr = tbuf;
      for (auto b : tcp) tptr += sprintf(tptr, "%02X ", b);
      *tptr = '\0';
      ESP_LOGD(TAG, "RTU->TCP response: %s", tbuf);
      ESP_LOGD(TAG, "Response time: %ums", millis() - pending_request_.start_time);
    }

    send(pending_request_.client_fd, tcp.data(), tcp.size(), 0);
    pending_request_.active = false;
    response.clear();

  } else if (millis() - pending_request_.start_time > 1000) {
    ESP_LOGW(TAG, "Modbus timeout: no valid response received.");
    pending_request_.response.clear();
    pending_request_.active = false;
  } else {
    this->set_timeout("modbus_rx_poll", 10, [this]() { this->poll_uart_response_(); });
  }
}

void ModbusBridgeComponent::update() {}

void ModbusBridgeComponent::append_crc(std::vector<uint8_t> &data) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; i++) {
      if (crc & 1) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  data.push_back(crc & 0xFF);
  data.push_back((crc >> 8) & 0xFF);
}

}  // namespace modbus_bridge
}  // namespace esphome
