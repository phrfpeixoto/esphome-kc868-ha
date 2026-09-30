#include "kc868_ha_component.h"
#include "math.h"

namespace esphome {
  namespace kc868_ha {

    static const char* const TAG = "kc868_ha";

    void KC868HaComponent::setup() {
      this->last_rx_ms_ = millis();
      ESP_LOGD(TAG, "KC868HaComponent::setup");
    }

    void KC868HaComponent::enqueue_tx(KC868HaSwitch *output) {
      const uint8_t target = output->get_target_relay_controller_addr();
      for (const auto &pending : this->tx_queue_) {
        if (pending.target_relay_controller_addr == target) {
          ESP_LOGD(TAG, "TX coalesced for target %u", static_cast<unsigned>(target));
          return;
        }
      }
      this->tx_queue_.push_back({target, output});
      ESP_LOGD(TAG, "TX queued for target %u (%u pending)", static_cast<unsigned>(target),
               static_cast<unsigned>(this->tx_queue_.size()));
    }

    void KC868HaComponent::loop() {
      while (this->available() > 0) {
        uint8_t byte;
        if (!this->read_byte(&byte))
          break;
        this->last_rx_ms_ = millis();
        this->rx_buffer_.push_back(byte);
        if (this->rx_buffer_.size() < 21)
          continue;
        auto *data = this->rx_buffer_.data();

        ESP_LOGD(TAG, "uart bus receive %s", format_uart_data(data, 21));

        uint8_t crc_data[19] =  {
          data[0],data[1],data[2],data[3],
          data[4],data[5],data[6],data[7],
          data[8],data[9],data[10],data[11],
          data[12],data[13],data[14],data[15],
          data[16],data[17],data[18]};
        uint16_t crc = crc16(crc_data, sizeof(crc_data));
        uint8_t crc_h = static_cast<uint8_t>(crc & 0x00FF);
        uint8_t crc_l = static_cast<uint8_t>((crc & 0xFF00) >> 8);
        ESP_LOGD(TAG, "uart crc=%x:%x, calc crc=%x:%x", data[19], data[20], crc_h, crc_l);

        if (!(data[19] == crc_h && data[20] == crc_l)) {
          ESP_LOGW(TAG, "CRC check failed, discarding one RX byte");
          this->rx_buffer_.erase(this->rx_buffer_.begin());
          continue;
        }

        for (auto & element : this->binary_sensors_)
          {
            if (element->get_target_relay_controller_addr() == data[0] &&
                element->get_switch_adapter_addr() == data[3]) {

              for (int i = 7; i <= 17; i += 2) {
                if (data[i] == (element->get_bind_output() + 100)) {
                  if (data[i+1] == 1) {
                    element->publish_state(true);
                  } else if (data[i+1] == 2) {
                    element->publish_state(false);
                  }
                }
              }
            }
          }
        this->rx_buffer_.clear();
      }

      // Defer if a byte arrived after the RX loop or a read could not complete.
      if (this->available() > 0) {
        this->last_rx_ms_ = millis();
        return;
      }
      if (this->tx_queue_.empty())
        return;
      if (static_cast<uint32_t>(millis() - this->last_rx_ms_) < this->tx_quiet_time_ms_) {
        if (!this->tx_deferred_logged_) {
          ESP_LOGD(TAG, "TX deferred because bus is active");
          this->tx_deferred_logged_ = true;
        }
        return;
      }

      if (this->has_tx_occurred_ &&
          static_cast<uint32_t>(millis() - this->last_tx_ms_) < this->tx_guard_time_ms_)
        return;

      // Construct the full bitmap now, so other outputs cannot be reverted by
      // a snapshot captured before their most recent desired state changed.
      const auto pending = this->tx_queue_.front();
      auto *output = pending.representative_output;
      auto frame = output->build_tx_frame(output->state);
      this->tx_queue_.erase(this->tx_queue_.begin());
      ESP_LOGD(TAG, "TX sending after quiet period");
      this->write_array(frame.data(), frame.size());
      this->flush();
      this->last_tx_ms_ = millis();
      this->has_tx_occurred_ = true;
      ESP_LOGD(TAG, "TX guard started (%u ms)", static_cast<unsigned>(this->tx_guard_time_ms_));
      ESP_LOGD(TAG, "uart bus send %s", format_uart_data(frame.data(), frame.size()));
      this->tx_deferred_logged_ = false;
    }

    void KC868HaComponent::dump_config(){
      ESP_LOGCONFIG(TAG, "KC868HaComponent::dump_config");
      ESP_LOGCONFIG(TAG, "KC868-HA code version: 2026.09.29.1");
      ESP_LOGCONFIG(TAG, "  TX quiet time: %u ms", static_cast<unsigned>(this->tx_quiet_time_ms_));
      ESP_LOGCONFIG(TAG, "  TX guard time: %u ms", static_cast<unsigned>(this->tx_guard_time_ms_));
    }

    void KC868HaBinarySensor::setup() {
      ESP_LOGD(TAG, "KC868HaBinarySensor::setup");
      this->publish_initial_state(false);
    }
    void KC868HaBinarySensor::dump_config(){
      ESP_LOGCONFIG(TAG, "KC868HaBinarySensor::dump_config");
    }

    void KC868HaSwitch::setup() {
      ESP_LOGD(TAG, "KC868HaSwitch::setup");

      bool initial_state = this->get_initial_state_with_restore_mode().value_or(false);

      if (initial_state) {
        this->turn_on();
      } else {
        this->turn_off();
      }
    }
    void KC868HaSwitch::dump_config(){
      ESP_LOGCONFIG(TAG, "KC868HaSwitch::dump_config");
    }

    void KC868HaSwitch::write_state(bool state) {
      this->parent_->enqueue_tx(this);
      this->publish_state(state);
    }

    std::vector<uint8_t> KC868HaSwitch::build_tx_frame(bool state) {

      uint8_t data[21] = {  this->get_target_relay_controller_addr(), 0x03, 0x12, 0x55, 0xBB,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

      int channel = this->get_bind_output();
      int byteIndex = (channel - 1) / 8; // determine which byte to modify
      int bitPosition = (channel - 1) % 8; // determine which bit to modify

      for (auto &element : *(this->switches_)) {
        if ((element->get_target_relay_controller_addr() == this->get_target_relay_controller_addr()) &&
            (element->get_bind_output() != this->get_bind_output())) {
          int elementChannel = element->get_bind_output();
          int elementByteIndex = (elementChannel - 1) / 8;
          int elementBitPosition = (elementChannel - 1) % 8;
          if (element->state == true) {
            data[20 - elementByteIndex] |= (1 << elementBitPosition); // set the bit
          } else {
            data[20 - elementByteIndex] &= ~(1 << elementBitPosition); // clear the bit
          }
        }
      }

      if (state == true) {
        data[20 - byteIndex] |= (1 << bitPosition); // set the bit
      } else {
        data[20 - byteIndex] &= ~(1 << bitPosition); // clear the bit
      }

      uint16_t crc = crc16(data, sizeof(data));
      uint8_t crc_h = static_cast<uint8_t>(crc & 0x00FF);
      uint8_t crc_l = static_cast<uint8_t>((crc & 0xFF00) >> 8);
      uint8_t uart_data[23] = {data[0], data[1], data[2], data[3],
                               data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11],
                               data[12], data[13], data[14], data[15], data[16], data[17], data[18], data[19], data[20],
                               crc_h, crc_l};

      return std::vector<uint8_t>(uart_data, uart_data + sizeof(uart_data));
    }

    uint16_t crc16(uint8_t *data, uint8_t length) {
      uint16_t crc = 0xFFFF;

      for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 8; j > 0; j--) {
          if (crc & 0x0001) {
            crc = (crc >> 1) ^ 0xA001;
          } else {
            crc >>= 1;
          }
        }
      }

      return crc;
    }

    char* format_uart_data(uint8_t *uart_data, int length) {
      static char str[256] = {0};  // Output buffer
      char tmp[10];  // Temporary buffer

      str[0] = '\0';  // Clear the buffer
      for (int i = 0; i < length; i++) {
        sprintf(tmp, "%x:", uart_data[i]);
        strcat(str, tmp);  // Append to str
      }

      str[strlen(str)-1] = '\0';  // Replace the last colon with a null terminator

      return str;
    }


  }
}
