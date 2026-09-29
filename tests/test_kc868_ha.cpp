#include "kc868_ha_component.h"
#include <cassert>
#include <iostream>
using namespace esphome;
using namespace esphome::kc868_ha;

class TestComponent : public KC868HaComponent {
 public:
  using KC868HaComponent::KC868HaComponent;
  size_t queued() const { return tx_queue_.size(); }
  size_t buffered() const { return rx_buffer_.size(); }
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
    f.tick(100); f.tick(199); assert(f.uart.tx.size()==1);
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
  { // The guard begins after flush and applies even with zero RX quiet time.
    Fixture f; f.component.set_tx_quiet_time(0); f.uart.flush_duration=24;
    f.outputs[0].write_state(true); f.outputs[1].write_state(true);
    f.tick(0); assert(now==24 && f.uart.tx.size()==1);
    f.tick(123); assert(f.uart.tx.size()==1);
    f.tick(124); assert(f.uart.tx.size()==2);
  }
  { // A configured guard is measured from flush completion, independently of RX quiet time.
    Fixture f; f.component.set_tx_guard_time(250); f.uart.flush_duration=24;
    f.outputs[0].write_state(true); f.outputs[1].write_state(true);
    f.tick(100); assert(now==124 && f.uart.tx.size()==1);
    f.receive(event(1)); f.tick(150); assert(f.sensor.updates==1);
    f.tick(250); f.tick(373); assert(f.uart.tx.size()==1);
    f.tick(374); assert(f.uart.tx.size()==2);
  }
  { // RX during the guard is delivered immediately and extends the RX wait.
    Fixture f; f.component.set_tx_quiet_time(250);
    f.outputs[0].write_state(true); f.outputs[1].write_state(true); f.tick(250);
    f.receive(event(1)); f.tick(300); assert(f.sensor.state && f.sensor.updates==1);
    f.receive(event(2)); f.tick(340); assert(!f.sensor.state && f.sensor.updates==2);
    f.tick(589); assert(f.uart.tx.size()==1); f.tick(590); assert(f.uart.tx.size()==2);
  }
  { // Noise followed by fragmented and consecutive valid frames resynchronizes.
    Fixture f; auto press=event(1); f.receive({255,255,17});
    f.receive(std::vector<uint8_t>(press.begin(),press.begin()+10)); f.tick(1);
    assert(f.sensor.updates==0 && f.component.buffered()==13);
    f.receive(std::vector<uint8_t>(press.begin()+10,press.end())); f.receive(event(2)); f.tick(2);
    assert(f.sensor.updates==2 && !f.sensor.state && f.component.buffered()==0);
  }
  { // Corruption, a deleted byte, and an inserted byte must not lose following frames.
    for(int mode=0;mode<3;mode++) {
      Fixture f; auto broken=event(1);
      if(mode==0) broken[19]^=1;
      if(mode==1) broken.erase(broken.begin()+8);
      if(mode==2) broken.insert(broken.begin()+8,255);
      f.receive(broken); f.receive(event(2)); f.tick(1);
      assert(f.sensor.updates==1 && !f.sensor.state && f.component.buffered()==0);
    }
  }
  { // Persistent noise uses bounded storage; partial RX restarts the quiet timer.
    Fixture f; f.outputs[0].write_state(true);
    f.receive(std::vector<uint8_t>(4096,255)); f.tick(50);
    assert(f.component.buffered()==20 && f.uart.tx.empty());
    f.receive({255}); f.tick(149); f.tick(248); assert(f.uart.tx.empty());
    f.tick(249); assert(f.uart.tx.size()==1);
  }
  { // RX callbacks can enqueue/coalesce commands without disrupting parsing.
    Fixture f; f.sensor.on_state=[&](bool state){f.outputs[0].write_state(state);};
    f.receive(event(1)); f.receive(event(2)); f.tick(100);
    assert(f.sensor.updates==2 && f.component.queued()==1 && f.uart.tx.empty());
    f.tick(200); assert(f.uart.tx.size()==1 && f.uart.tx[0][20]==0);
  }
  { // Guard and RX elapsed-time arithmetic survive millis rollover.
    Fixture f; f.component.set_tx_quiet_time(0);
    f.outputs[0].write_state(true); f.outputs[1].write_state(true);
    f.tick(0xfffffff0U); f.tick(83); assert(f.uart.tx.size()==1);
    f.tick(84); assert(f.uart.tx.size()==2);
  }
  { // RX observed before rollover still requires the complete quiet period.
    Fixture f; f.outputs[0].write_state(true);
    f.receive({255}); f.tick(0xfffffff0U);
    f.tick(83); assert(f.uart.tx.empty());
    f.tick(84); assert(f.uart.tx.size()==1);
  }
  std::cout << "PASS: coalescing, fresh bitmaps, queue bounds, TX guard, RX quiet time, CRC recovery, callbacks, rollover\n";
}
