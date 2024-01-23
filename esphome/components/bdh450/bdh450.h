#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/time.h"

#include <vector>

namespace esphome {
namespace bdh450 {

class BDH450Component;


class BDH450Component : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  void set_clk_pin(InternalGPIOPin *pin) { this->clk_pin_ = pin; }
  void set_stb_pin(InternalGPIOPin *pin) { this->stb_pin_ = pin; }
  void set_dio_pin(GPIOPin *pin) { this->dio_pin_ = pin; }
  void set_power_sensor(binary_sensor::BinarySensor *sensor) { the_power_sensor_ = sensor; }
  void set_tank_sensor(binary_sensor::BinarySensor *sensor) { the_tank_sensor_ = sensor; }
  void set_defrost_sensor(binary_sensor::BinarySensor *sensor) { the_defrost_sensor_ = sensor; }
  void set_fan_mode_sensor(text_sensor::TextSensor *sensor) { the_fan_mode_ = sensor; }
  void set_power_sensor(sensor::Sensor *sensor) { the_humidity_sensor_ = sensor; }

  bool is_on();
  bool is_off();

  void loop() override;

 protected:
  void listen_(bool enable_listen);
  uint8_t get_digit_(uint8_t my_bits);

  void process_byte_();

  void IRAM_ATTR clock_read_();
  void IRAM_ATTR strobe_active_();

  InternalGPIOPin *clk_pin_;
  InternalGPIOPin *stb_pin_;
  GPIOPin *dio_pin_;

  binary_sensor::BinarySensor *the_power_sensor_{nullptr};
  binary_sensor::BinarySensor *the_tank_sensor_{nullptr};
  binary_sensor::BinarySensor *the_defrost_sensor_{nullptr};
  text_sensor::TextSensor *the_fan_mode_{nullptr};
  sensor::Sensor *the_humidity_sensor_{nullptr};


  bool is_strobe_active_ = false; // If stroibe is active we're reading data
  bool process_byte_ = false; // Whether the main loop should process the data read

  uint8_t ary_idx_ = 0; // where we're writing to the array
  uint8_t byte_cache_[] = new uint8_t[5]; // small array in case we get data faster than we're processing it

  int bits_read_ = 0; // how many bits read so far, we kick it at 8 no matter what
  uint8_t workingy_byte_ = 0; // Last data byte read from the serial

  uint8_t _wait_byte_idx_ = 0; // which byte of the 6 are we processing
  bool getting_display_ = false; // if we're parsing the 0xC0 command that pushes data to the display

  uint8_t digit_ones_ = 0; // temp hold ones digit
  uint8_t digit_tens_ = 0; // temp hold tens digit
  bool already_listening_ = false; // we started listening for an update

};

}  // namespace bdh450
}  // namespace esphome
