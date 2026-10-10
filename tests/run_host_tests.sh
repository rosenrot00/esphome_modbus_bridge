#!/usr/bin/env bash
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
build=$(mktemp -d)
trap 'rm -rf "$build"' EXIT
for header in esphome/core/component.h esphome/core/hal.h esphome/core/log.h \
  esphome/core/version.h esphome/core/automation.h esphome/components/uart/uart.h \
  esphome/components/switch/switch.h esphome/components/network/util.h \
  lwip/sockets.h lwip/tcpip.h lwip/dns.h ESP8266WiFi.h; do
  mkdir -p "$build/include/$(dirname "$header")"
  printf '#include "host_runtime.h"\n' > "$build/include/$header"
done
mkdir -p "$build/include/esphome/components/modbus"
printf '#include "host_modbus.h"\n' > "$build/include/esphome/components/modbus/modbus.h"
for platform in ESP32 ESP8266; do
  "${CXX:-c++}" -std=c++17 -g -O1 -fsanitize=address,undefined \
    -Wno-deprecated-declarations -DUSE_"$platform" -I"$build/include" -I"$root/tests" \
    "$root/tests/test_bridge.cpp" -o "$build/test_$platform"
  "$build/test_$platform"
  "${CXX:-c++}" -std=c++20 -g -O1 -fsanitize=address,undefined \
    -Wno-deprecated-declarations -DUSE_"$platform" -DUSE_MODBUS_BRIDGE_HUB \
    -I"$build/include" -I"$root/tests" "$root/tests/test_bridge.cpp" -o "$build/test_${platform}_hub"
  "$build/test_${platform}_hub"
done
