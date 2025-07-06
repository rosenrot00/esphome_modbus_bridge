#include "modbus_bridge.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <lwip/sockets.h>
#include <lwip/netdb.h>
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
  this->sock_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
  if (this->sock_ < 0) {
    ESP_LOGE(TAG, "Socket creation failed");
    return;
  }

  fcntl(this->sock_, F_SETFL, O_NONBLOCK);

  sockaddr_in server_addr{};
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(this->tcp_port_);
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (::bind(this->sock_, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Socket bind failed");
    close(this->sock_);
    return;
  }

  if (::listen(this->sock_, 4) < 0) {
    ESP_LOGE(TAG, "Socket listen failed");
    close(this->sock_);
    return;
  }

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
      maxfd = std::max(maxfd, c.fd);
    }
  }

  timeval timeout{0, 10000};
  int sel = select(maxfd + 1, &read_fds, nullptr, nullptr, &timeout);
  if (sel < 0) return;

  if (FD_ISSET(this->sock_, &read_fds)) {
    int newfd = accept(this->sock_, nullptr, nullptr);
    if (newfd >= 0) {
      fcntl(newfd, F_SETFL, O_NONBLOCK);
      bool assigned = false;
      for (auto &c : this->clients_) {
        if (c.fd < 0) {
          c.fd = newfd;
          ESP_LOGD(TAG, "Client connected: %d", newfd);
          assigned = true;
          break;
        }
      }
      if (!assigned) {
        ESP_LOGW(TAG, "Max clients reached, rejecting new client");
        close(newfd);
      }
    }
  }

  for (auto &c : this->clients_) {
    if (c.fd >= 0 && FD_ISSET(c.fd, &read_fds)) {
      size_t buffer_size = uart_->get_rx_buffer_size() + 6;
      std::vector<uint8_t> tcp_buffer(buffer_size);
      int r = recv(c.fd, tcp_buffer.data(), tcp_buffer.size(), 0);
      if (r <= 0) {
        ESP_LOGD(TAG, "Client disconnected: %d", c.fd);
        close(c.fd);
        c.fd = -1;
        continue;
      }
      if (r < 8) continue;

      uint16_t len = (tcp_buffer[4] << 8) | tcp_buffer[5];
      if (r < len + 6) continue;

      std::vector<uint8_t> rtu(tcp_buffer.begin() + 6, tcp_buffer.begin() + 6 + len);
      append_crc(rtu);

      if (debug_) log_hex("TCP->RTU", rtu);

      uart_->write_array(rtu);
      uart_->flush();

      pending_request_.active = true;
      pending_request_.client_fd = c.fd;
      pending_request_.header[0] = tcp_buffer[0];
      pending_request_.header[1] = tcp_buffer[1];
      pending_request_.header[2] = tcp_buffer[2];
      pending_request_.header[3] = tcp_buffer[3];
      pending_request_.start_time = millis();
      pending_request_.last_size = 0;
      pending_request_.no_data_counter = 0;
      pending_request_.response.clear();

      this->set_interval("modbus_uart_poll", 5, [this]() { poll_uart_response_(); });
      break;
    }
  }
}

void ModbusBridgeComponent::poll_uart_response_() {
  while (uart_->available()) {
    uint8_t b;
    uart_->read_byte(&b);
    pending_request_.response.push_back(b);
    pending_request_.no_data_counter = 0;
  }

  if (++pending_request_.no_data_counter >= 2) {
    if (debug_) log_hex("RTU->TCP", pending_request_.response);

    send_response_tcp();
    pending_request_.active = false;
    cancel_interval("modbus_uart_poll");
  }

  if (millis() - pending_request_.start_time > 1000) {
    ESP_LOGW(TAG, "Modbus timeout");
    pending_request_.active = false;
    cancel_interval("modbus_uart_poll");
  }
}

void ModbusBridgeComponent::send_response_tcp() {
  std::vector<uint8_t> &resp = pending_request_.response;
  uint16_t pdulen = resp.size() - 3;
  std::vector<uint8_t> tcp_resp;
  tcp_resp.insert(tcp_resp.end(), pending_request_.header, pending_request_.header + 4);
  tcp_resp.push_back(0);
  tcp_resp.push_back(pdulen + 1);
  tcp_resp.push_back(resp[0]);
  tcp_resp.insert(tcp_resp.end(), resp.begin() + 1, resp.end() - 2);

  send(pending_request_.client_fd, tcp_resp.data(), tcp_resp.size(), 0);
}

void ModbusBridgeComponent::log_hex(const char *prefix, const std::vector<uint8_t> &data) {
  char buf[data.size() * 3 + 1];
  char *ptr = buf;
  for (auto b : data) ptr += sprintf(ptr, "%02X ", b);
  *ptr = '\0';
  ESP_LOGD(TAG, "%s: %s", prefix, buf);
}

void ModbusBridgeComponent::append_crc(std::vector<uint8_t> &data) {
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= b;
    for (int i = 0; i < 8; i++)
      crc = crc & 1 ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  data.push_back(crc & 0xFF);
  data.push_back((crc >> 8) & 0xFF);
}

}  // namespace modbus_bridge
}  // namespace esphome
