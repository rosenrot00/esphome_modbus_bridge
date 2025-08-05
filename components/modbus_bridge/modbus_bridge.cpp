
#include <vector>
#include <memory>
#include <functional>
#include <initializer_list>
#include <cstring>
#include <cmath>

#ifdef USE_ESP8266
#include <ESP8266WiFi.h>
#elif defined(USE_ESP32)
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

ModbusBridgeComponent::ModbusBridgeComponent() {}

void ModbusBridgeComponent::set_rtu_response_timeout(uint32_t timeout) {
  this->rtu_response_timeout_ms_ = timeout;
}

void ModbusBridgeComponent::setup() {
#if defined(USE_ESP8266)
  this->server_ = WiFiServer(this->tcp_port_);
  this->server_.begin();
  this->set_interval("tcp_poll", this->tcp_poll_interval_ms_, [this]() {
    this->check_tcp_sockets_esp8266_();
  });
#elif defined(USE_ESP32)
  this->set_interval("tcp_server_and_network_check", 1000, [this]() {
    if (this->sock_ < 0 && network::is_connected()) {
      ESP_LOGI(TAG, "IP available – initializing TCP server");
      this->initialize_tcp_server_();
    } else if (this->sock_ >= 0 && !network::is_connected()) {
      ESP_LOGW(TAG, "Lost network IP – closing TCP server");
      close(this->sock_);
      this->sock_ = -1;
    }
  });

  this->set_interval("tcp_poll", this->tcp_poll_interval_ms_, [this]() {
    this->check_tcp_sockets_esp32_();
  });
#endif

  this->rtu_inactivity_timeout_ms_ = static_cast<uint32_t>(std::ceil((10000.0 / this->uart_->get_baud_rate()) * 3.5 + 1));
  this->temp_buffer_.resize(this->uart_->get_rx_buffer_size() + 6);
  this->rtu_poll_interval_ms_ = this->rtu_inactivity_timeout_ms_ + 2;
  this->polling_active_ = false;
  if (this->rtu_response_timeout_ms_ == 0) {
    ESP_LOGW(TAG, "RTU response timeout not set via YAML – falling back to 100 ms.");
    this->rtu_response_timeout_ms_ = 100;
  }
}

#ifdef USE_ESP32
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
#endif

#ifdef USE_ESP32
void ModbusBridgeComponent::check_tcp_sockets_esp32_() {
  if (this->sock_ < 0 || pending_request_.active) return;

  fd_set read_fds;
  FD_ZERO(&read_fds);
  FD_SET(this->sock_, &read_fds);
  int maxfd = this->sock_;

  uint32_t now = millis();

  for (auto &c : this->clients_) {
    if (c.fd >= 0) {
      if (now - c.last_activity > this->tcp_client_timeout_ms_) {
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
      bool accepted = false;
      for (auto &c : this->clients_) {
        if (c.fd < 0) {
          c.fd = newfd;
          c.last_activity = millis();
          ESP_LOGI(TAG, "Client connected: %d", newfd);
          accepted = true;
          break;
        }
      }
      if (!accepted) {
        ESP_LOGW(TAG, "No slot for new client, closing fd %d", newfd);
        close(newfd);
      }
    }
  }

  for (auto &c : this->clients_) {
    if (c.fd >= 0 && FD_ISSET(c.fd, &read_fds)) {
      int r = recv(c.fd, this->temp_buffer_.data(), this->temp_buffer_.size(), 0);
      if (r == 0) {
        //ESP_LOGW(TAG, "Client %d recv == 0 (no data), keeping connection", c.fd);
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

      uint16_t len = (this->temp_buffer_[4] << 8) | this->temp_buffer_[5];
      if (len > this->temp_buffer_.size() - 6) {
        ESP_LOGW(TAG, "Invalid Modbus length field: %d", len);
        continue;
      }
      if (r < 6 + len) continue;

      uint8_t uid = this->temp_buffer_[6];
      std::vector<uint8_t> rtu;
      rtu.push_back(uid);
      rtu.insert(rtu.end(), this->temp_buffer_.begin() + 7, this->temp_buffer_.begin() + 6 + len);
      append_crc(rtu);

      if (this->debug_) {
        ESP_LOGD(TAG, "TCP->RTU UID: %d, FC: 0x%02X, LEN: %d", uid, rtu[1], len);
        char buf[rtu.size() * 3 + 1];
        char *ptr = buf;
        for (auto b : rtu) ptr += sprintf(ptr, "%02X ", b);
        *ptr = 0;
        ESP_LOGD(TAG, "RTU send: %s", buf);
      }
      
      this->uart_->flush();
      this->uart_->write_array(rtu);
      pending_request_.client_fd = c.fd;
      memcpy(pending_request_.header, this->temp_buffer_.data(), 7);
      pending_request_.response.clear();
      pending_request_.response.reserve(this->uart_->get_rx_buffer_size());
      pending_request_.active = true;
      pending_request_.start_time = millis();
      pending_request_.last_size = 0;
      pending_request_.last_change = millis();

      this->start_uart_polling_();
      c.last_activity = now;
      break;
    }
  }
}
#endif

#ifdef USE_ESP8266
void ModbusBridgeComponent::check_tcp_sockets_esp8266_() {
  WiFiClient new_client = this->server_.accept();
  if (new_client) {
    if (this->clients_.size() < 4) {
      new_client.setTimeout(10);
      this->clients_.push_back(new_client);
      ESP_LOGI(TAG, "New client connected");
    } else {
      new_client.stop();
      ESP_LOGW(TAG, "Max clients reached");
    }
  }

  for (auto it = this->clients_.begin(); it != this->clients_.end(); ) {
    if (!it->connected()) {
      it->stop();
      it = this->clients_.erase(it);
      continue;
    }

    if (it->available() >= 7) {
      int r = it->readBytes(this->temp_buffer_.data(), this->temp_buffer_.size());
      // Optional: hier gemeinsame Parse-/Handling-Logik extrahieren
    }

    ++it;
  }
}
#endif

void ModbusBridgeComponent::start_uart_polling_() {
  if (this->polling_active_) return;

  this->set_timeout("modbus_rx_poll", this->rtu_poll_interval_ms_, [this]() { poll_uart_response_(); });
  this->polling_active_ = true;
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

  if (current_size == 0) {
    if (millis() - pending_request_.start_time > this->rtu_response_timeout_ms_) {
      ESP_LOGW(TAG, "Modbus timeout: no response received (no first byte).");
      end_pending_request_();
      return;
    }

    this->set_timeout("modbus_rx_poll", this->rtu_poll_interval_ms_, [this]() { poll_uart_response_(); });
    return;
  }

  if (current_size < 3) {
    ESP_LOGW(TAG, "Invalid response length: too short");
    end_pending_request_();
    return;
  }

  if (current_size > pending_request_.last_size) {
    pending_request_.last_size = current_size;
    pending_request_.last_change = millis();
  }

  if (millis() - pending_request_.last_change > this->rtu_inactivity_timeout_ms_) {
    if (this->debug_) {
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

    if (this->debug_) {
      std::string tcp_debug;
      for (uint8_t b : tcp_response)
        tcp_debug += str_snprintf("%02X ", 3, b);
      ESP_LOGD(TAG, "RTU->TCP response: %s", tcp_debug.c_str());
      ESP_LOGD(TAG, "Response time: %ums", millis() - pending_request_.start_time);
    }

    // Plattform-spezifischer TCP-Response
#ifdef USE_ESP8266
    for (auto &client : this->clients_) {
      if (client.connected()) {
        client.write(tcp_response.data(), tcp_response.size());
        break;
      }
    }
#elif defined(USE_ESP32)
    send(pending_request_.client_fd, tcp_response.data(), tcp_response.size(), 0);
#endif
    end_pending_request_();
    return;
  }

  if (millis() - pending_request_.start_time > this->rtu_response_timeout_ms_) {
    ESP_LOGW(TAG, "Modbus timeout: response incomplete.");
    end_pending_request_();
    return;
  }

  this->set_timeout("modbus_rx_poll", this->rtu_poll_interval_ms_, [this]() { poll_uart_response_(); });
}

void ModbusBridgeComponent::end_pending_request_() {
  if (pending_request_.response.capacity() > this->uart_->get_rx_buffer_size()) {
    std::vector<uint8_t>().swap(pending_request_.response);
    pending_request_.response.reserve(this->uart_->get_rx_buffer_size());
  } else {
    pending_request_.response.clear();
  }
  pending_request_.active = false;
  polling_active_ = false;
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

void ModbusBridgeComponent::set_debug(bool debug) {
  this->debug_ = debug;
  ESP_LOGI(TAG, "Debug mode %s", debug ? "enabled" : "disabled");
}


}  // namespace modbus_bridge
}  // namespace esphome
