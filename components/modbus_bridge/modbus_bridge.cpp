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

  this->set_interval("tcp_poll", this->tcp_poll_interval_ms_, [this]() {
    this->check_tcp_sockets_();
  });

  this->polling_active_ = false;
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

void ModbusBridgeComponent::check_tcp_sockets_() {
  if (this->sock_ < 0 || pending_request_.active) return;

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(this->sock_, &read_fds);
  int maxfd = this->sock_;

  uint32_t now = millis();

  for (auto &c : this->clients_) {
    if (c.fd >= 0) {
      if (now - c.last_activity > this->client_timeout_ms_) {
        ESP_LOGW(TAG, "Client timeout: closing fd %d", c.fd);
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
    int newfd = accept(this->sock_, NULL, NULL);
    if (newfd >= 0) {
      fcntl(newfd, F_SETFL, O_NONBLOCK);
      for (auto &c : this->clients_) {
        if (c.fd < 0) {
          c.fd = newfd;
          c.last_activity = millis();
          ESP_LOGD(TAG, "Client connected: %d", newfd);
          break;
        }
      }
    }
  }

  for (auto &c : this->clients_) {
    if (c.fd >= 0 && FD_ISSET(c.fd, &read_fds)) {
      constexpr size_t max_buffer = 256 + 6;
      uint8_t buffer[max_buffer];
      int r = recv(c.fd, buffer, sizeof(buffer), 0);
      if (r <= 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
          ESP_LOGW(TAG, "Client %d disconnected or error", c.fd);
          close(c.fd);
          c.fd = -1;
        }
        continue;
      }
      if (r < 7) {
        ESP_LOGW(TAG, "Received too-short frame (%d bytes) from fd %d", r, c.fd);
        continue;
      }

      uint16_t len = (buffer[4] << 8) | buffer[5];
      if (r < 6 + len) continue;

      uint8_t uid = buffer[6];
      std::vector<uint8_t> rtu;
      rtu.push_back(uid);
      rtu.insert(rtu.end(), buffer + 7, buffer + 6 + len);
      append_crc(rtu);

      if (debug_) {
        ESP_LOGD(TAG, "TCP->RTU UID: %d, FC: 0x%02X, LEN: %d", uid, rtu[1], len);
        char buf[rtu.size() * 3 + 1];
        char *ptr = buf;
        for (auto b : rtu) ptr += sprintf(ptr, "%02X ", b);
        *ptr = 0;
        ESP_LOGD(TAG, "RTU send: %s", buf);
      }

      this->uart_->write_array(rtu);
      pending_request_.client_fd = c.fd;
      memcpy(pending_request_.header, buffer, 7);
      pending_request_.response.clear();
      pending_request_.active = true;
      pending_request_.start_time = millis();
      pending_request_.last_size = 0;
      pending_request_.no_data_counter = 0;

      this->start_uart_polling_();
      c.last_activity = now;
      break;
    }
  }
}

}  // namespace modbus_bridge
}  // namespace esphome
