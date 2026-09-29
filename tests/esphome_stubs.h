// Minimal ESPHome doubles for host-side protocol regression tests.

#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <vector>
#include <deque>
#include <optional>
#include <functional>
#define ESP_LOGD(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGCONFIG(...) ((void)0)
namespace esphome {
inline uint32_t now;
inline uint32_t millis() { return now; }
class Component { public: virtual void setup() {} virtual void loop() {} virtual void dump_config() {} };
namespace binary_sensor { class BinarySensor { public: bool state=false; int updates=0; std::function<void(bool)> on_state; void publish_state(bool s) {state=s; updates++; if(on_state) on_state(s);} void publish_initial_state(bool s) {state=s;} }; }
namespace switch_ { class Switch { public: bool state=false; virtual void write_state(bool)=0; std::optional<bool> get_initial_state_with_restore_mode() {return false;} void publish_state(bool s) {state=s;} void turn_on() {write_state(true);} void turn_off() {write_state(false);} }; }
namespace uart {
class UARTComponent { public: std::deque<uint8_t> rx; std::vector<std::vector<uint8_t>> tx; int flushes=0; uint32_t flush_duration=0; void write_array(const uint8_t *d,size_t n) {tx.emplace_back(d,d+n);} void flush() {flushes++; now += flush_duration;} };
class UARTDevice { public: UARTComponent *u; UARTDevice(UARTComponent *p):u(p) {} size_t available() {return u->rx.size();} bool read_byte(uint8_t *b) {if(u->rx.empty()) return false; *b=read(); return true;} int read() {auto c=u->rx.front();u->rx.pop_front();return c;} void write_array(const uint8_t *d,size_t n) {u->write_array(d,n);} void flush() {u->flush();} };
}
}
