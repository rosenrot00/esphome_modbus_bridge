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
      size_t max_buffer = this->uart_->get_rx_buffer_size() + 6;
      std::vector<uint8_t> buffer(max_buffer);
      int r = recv(c.fd, buffer.data(), buffer.size(), 0);
      if (r == 0) {
        ESP_LOGI(TAG, "Client %d disconnected cleanly", c.fd);
        close(c.fd);
        c.fd = -1;
        continue;
      }
      if (r < 0) {
        if (errno != EWOULDBLOCK && errno != EAGAIN) {
          ESP_LOGW(TAG, "Client %d socket error: %s", c.fd, strerror(errno));
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
      rtu.insert(rtu.end(), buffer.begin() + 7, buffer.begin() + 6 + len);
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
      memcpy(pending_request_.header, buffer.data(), 7);
      pending_request_.response.clear();
      pending_request_.response.reserve(this->uart_->get_rx_buffer_size());
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

void ModbusBridgeComponent::start_uart_polling_() {
  if (this->polling_active_) return;
  this->polling_active_ = true;

  this->set_timeout("modbus_rx_poll", 10, [this]() { poll_uart_response_(); });
}

void ModbusBridgeComponent::poll_uart_response_() {
  if (!pending_request_.active) {
    polling_active_ = false;
    return;
  }

  while (this->uart_->available()) {
    uint8_t byte;
    if (this->uart_->read_byte(&byte)) {
      pending_request_.response.push_back(byte);
    }
  }

  size_t current_size = pending_request_.response.size();
  if (current_size == pending_request_.last_size) {
    pending_request_.no_data_counter++;
  } else {
    pending_request_.no_data_counter = 0;
    pending_request_.last_size = current_size;
  }

  if (pending_request_.no_data_counter >= 2) {
    if (debug_) {
      std::string debug_output;
      for (uint8_t b : pending_request_.response)
        debug_output += str_snprintf("%02X ", 3, b);
      ESP_LOGD(TAG, "RTU recv (%d bytes): %s", (int)current_size, debug_output.c_str());
    }

    std::vector<uint8_t> tcp_response;
    uint16_t pdu_length = current_size - 3;
    tcp_response.insert(tcp_response.end(), pending_request_.header, pending_request_.header + 4);
    tcp_response.push_back(0);
    tcp_response.push_back(pdu_length + 1);
    tcp_response.push_back(pending_request_.response[0]);
    tcp_response.insert(tcp_response.end(), pending_request_.response.begin() + 1, pending_request_.response.end() - 2);

    if (debug_) {
      std::string tcp_debug;
      for (uint8_t b : tcp_response)
        tcp_debug += str_snprintf("%02X ", 3, b);
      ESP_LOGD(TAG, "RTU->TCP response: %s", tcp_debug.c_str());
      ESP_LOGD(TAG, "Response time: %ums", millis() - pending_request_.start_time);
    }

    send(pending_request_.client_fd, tcp_response.data(), tcp_response.size(), 0);
    pending_request_.response.clear();
    pending_request_.active = false;
    polling_active_ = false;
    return;
  }

  if (millis() - pending_request_.start_time > 1000) {
    ESP_LOGW(TAG, "Modbus timeout: no valid response received.");
    pending_request_.response.clear();
    pending_request_.active = false;
    polling_active_ = false;
    return;
  }

  this->set_timeout("modbus_rx_poll", 10, [this]() { poll_uart_response_(); });
}

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

}  // namespace modbus_bridge
}  // namespace esphome
