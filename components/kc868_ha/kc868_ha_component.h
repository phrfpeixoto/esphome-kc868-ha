#pragma once

#include <vector>

#include "esphome.h"

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
  namespace kc868_ha {

    class KC868HaComponent;

    class KC868HaBinarySensor : public Component, public binary_sensor::BinarySensor {
    public:
      void set_target_relay_controller_addr(uint8_t addr) { this->target_relay_controller_addr_=addr; };
      uint8_t get_target_relay_controller_addr() { return this->target_relay_controller_addr_; };
      void set_switch_adapter_addr(uint8_t addr) { this->switch_adapter_addr_ = addr; };
      uint8_t get_switch_adapter_addr() { return this->switch_adapter_addr_; };
      void set_bind_output(uint8_t bind_output) { this->bind_output_ = bind_output; };
      uint8_t get_bind_output() { return this->bind_output_; };
      void setup() override;
      void dump_config() override;

    protected:
      uint8_t target_relay_controller_addr_;
      uint8_t switch_adapter_addr_;
      uint8_t bind_output_;
    };

    class KC868HaSwitch : public Component, public switch_::Switch {
    public:
      void set_parent(KC868HaComponent *parent) { this->parent_ = parent; }
      void set_uart(uart::UARTComponent *uartComponent) { this->uart_ = uartComponent;};
      void set_target_relay_controller_addr(uint8_t addr) { this->target_relay_controller_addr_=addr; };
      uint8_t get_target_relay_controller_addr() { return this->target_relay_controller_addr_; };
      void set_switch_adapter_addr(uint8_t addr) { this->switch_adapter_addr_ = addr; };
      uint8_t get_switch_adapter_addr() { return this->switch_adapter_addr_; };
      void set_bind_output(uint8_t bind_output) { this->bind_output_ = bind_output; };
      uint8_t get_bind_output() { return this->bind_output_; };

      void setup() override;
      void write_state(bool state) override;
      std::vector<uint8_t> build_tx_frame(bool state);
      void dump_config() override;
      void set_switches(std::vector<kc868_ha::KC868HaSwitch *>* switches_)  { this->switches_ = switches_; };

    protected:
      KC868HaComponent *parent_{nullptr};
      uart::UARTComponent *uart_;
      uint8_t target_relay_controller_addr_;
      uint8_t switch_adapter_addr_;
      uint8_t bind_output_;
      std::vector<kc868_ha::KC868HaSwitch *>* switches_;
    };

    class KC868HaComponent : public Component, public uart::UARTDevice {
    public:
      KC868HaComponent(uart::UARTComponent *uartComponent) : UARTDevice(uartComponent) {
        this->uart_ = uartComponent;
      }
      //void set_target_relay_controller_addr(uint8_t addr) { this->target_relay_controller_addr_=addr; };
      //uint8_t get_target_relay_controller_addr() { return this->target_relay_controller_addr_; };
      //void set_switch_adapter_addr(uint8_t addr) { this->switch_adapter_addr_=addr; };
      //uint8_t get_switch_adapter_addr() { return this->switch_adapter_addr_; };
      void register_binary_sensor(kc868_ha::KC868HaBinarySensor *obj)  { this->binary_sensors_.push_back(obj); };
      void register_switch(kc868_ha::KC868HaSwitch *obj)  {
        obj->set_parent(this);
        obj->set_uart(this->uart_);
        obj->set_switches(&(this->switches_));
        this->switches_.push_back(obj);
      };
      void setup() override;
      void loop() override;
      void dump_config() override;

      void enqueue_tx(KC868HaSwitch *output, bool state);
      void set_tx_quiet_time(uint32_t quiet_time_ms) { this->tx_quiet_time_ms_ = quiet_time_ms; }
      void set_tx_guard_time(uint32_t guard_time_ms) { this->tx_guard_time_ms_ = guard_time_ms; }

    protected:
      uint32_t tx_quiet_time_ms_{100};
      static constexpr size_t MAX_TX_QUEUE = 8;
      uint32_t tx_guard_time_ms_{100};
      struct PendingOutput {
        KC868HaSwitch *output;
        bool state;
      };
      uint32_t last_rx_ms_{0};
      uint32_t last_tx_ms_{0};
      bool tx_guard_active_{false};
      size_t pending_rx_bytes_{0};
      void observe_rx_activity_();
      bool tx_deferred_logged_{false};
      std::vector<PendingOutput> tx_queue_;
      uart::UARTComponent *uart_;
      uint8_t target_relay_controller_addr_;
      uint8_t switch_adapter_addr_;
      std::vector<kc868_ha::KC868HaBinarySensor *> binary_sensors_;
      std::vector<kc868_ha::KC868HaSwitch *> switches_;
    };

    uint16_t crc16(uint8_t *data, uint8_t length);
    char* format_uart_data(uint8_t *uart_data, int length);
  }
}
