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
      constexpr size_t max_buffer = 512;
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
      }

      this->uart_->write_array(rtu);
      pending_request_.client_fd = c.fd;
      memcpy(pending_request_.header, buffer, 7);
      pending_request_.response.clear();
      pending_request_.response.reserve(512);  // deutlich größerer Buffer
      pending_request_.active = true;
      pending_request_.start_time = millis();
      pending_request_.last_size = 0;
      pending_request_.no_data_counter = 0;

      this->start_uart_polling_();
      c.last_activity = now;
      ESP_LOGD(TAG, "Started UART polling, active: %d", polling_active_);
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

  size_t new_size = pending_request_.response.size();

  ESP_LOGD(TAG, "Polling active: %d, Pending active: %d, No data counter: %d, Buffer size: %d",
           polling_active_, pending_request_.active, pending_request_.no_data_counter, (int)new_size);

  if (new_size == pending_request_.last_size) {
    pending_request_.no_data_counter++;
  } else {
    pending_request_.no_data_counter = 0;
    pending_request_.last_size = new_size;
  }

  if (pending_request_.no_data_counter >= 2) {
    ESP_LOGD(TAG, "End of UART data detected.");

    std::vector<uint8_t> &response = pending_request_.response;
    std::vector<uint8_t> tcp;
    uint16_t pdulen = response.size() - 3;
    tcp.insert(tcp.end(), pending_request_.header, pending_request_.header + 4);
    tcp.push_back(0);
    tcp.push_back(pdulen + 1);
    tcp.push_back(response[0]);
    tcp.insert(tcp.end(), response.begin() + 1, response.end() - 2);

    send(pending_request_.client_fd, tcp.data(), tcp.size(), 0);
    pending_request_.active = false;
    ESP_LOGD(TAG, "Sent TCP response, stopped UART polling.");
    return;
  }

  if (millis() - pending_request_.start_time > this->modbus_response_timeout_ms_) {
    ESP_LOGW(TAG, "Modbus timeout: no valid response received.");
    pending_request_.response.clear();
    pending_request_.active = false;
    return;
  }
}
