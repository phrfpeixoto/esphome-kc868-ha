#include "kc868_ha_component.h"
#include <cassert>
#include <iostream>
using namespace esphome;
using namespace esphome::kc868_ha;

class TestComponent : public KC868HaComponent {
 public:
  using KC868HaComponent::KC868HaComponent;
  size_t queued() const { return tx_queue_.size(); }
  uint8_t first_output() const { return tx_queue_.front().output->get_bind_output(); }
};
struct Fixture {
  uart::UARTComponent uart;
  TestComponent component{&uart};
  KC868HaSwitch outputs[10];
  KC868HaBinarySensor sensor;
  Fixture() {
    now=0; component.setup();
    for (int i=0;i<10;i++) {
      outputs[i].set_target_relay_controller_addr(1);
      outputs[i].set_switch_adapter_addr(10);
      outputs[i].set_bind_output(i+1);
      component.register_switch(&outputs[i]);
    }
    sensor.set_target_relay_controller_addr(1);
    sensor.set_switch_adapter_addr(10);
    sensor.set_bind_output(1);
    component.register_binary_sensor(&sensor);
  }
  void tick(uint32_t t) {now=t; component.loop();}
  void receive(const std::vector<uint8_t> &bytes) {uart.rx.insert(uart.rx.end(),bytes.begin(),bytes.end());}
};
std::vector<uint8_t> event(uint8_t state) {
  std::vector<uint8_t> d(21); d[0]=1; d[3]=10; d[7]=101; d[8]=state;
  auto crc=crc16(d.data(),19); d[19]=crc&255; d[20]=crc>>8; return d;
}
int main() {
  { // Repeated transitions collapse; later frames cannot replay stale bitmaps.
    Fixture f;
    f.outputs[0].write_state(true); f.outputs[1].write_state(true);
    f.outputs[0].write_state(false);
    assert(f.component.queued()==2 && f.uart.tx.empty());
    f.tick(99); assert(f.uart.tx.empty());
    f.tick(100); assert(f.uart.tx.size()==1 && f.uart.tx[0][20]==2);
    f.tick(200); assert(f.uart.tx.size()==2 && f.uart.tx[1][20]==2);
    for(auto &frame:f.uart.tx) {
      assert(frame.size()==23); auto crc=crc16(frame.data(),21);
      assert(frame[21]==(crc&255) && frame[22]==(crc>>8));
    }
    assert(f.uart.flushes==2);
  }
  { // Adapter address is part of the key even for identical bound outputs.
    Fixture f; f.outputs[1].set_switch_adapter_addr(11); f.outputs[1].set_bind_output(1);
    f.outputs[0].write_state(true); f.outputs[1].write_state(true);
    f.outputs[0].write_state(false); assert(f.component.queued()==2);
  }
  { // Coalescing at capacity must not evict an unrelated output.
    Fixture f;
    for(int i=0;i<8;i++) f.outputs[i].write_state(true);
    f.outputs[0].write_state(false); assert(f.component.queued()==8 && f.component.first_output()==1);
    f.outputs[8].write_state(true); assert(f.component.queued()==8 && f.component.first_output()==2);
  }
  std::cout << "PASS: protocol regression tests\n";
}
