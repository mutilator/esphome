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
  void update() override;
  float get_setup_priority() const override;

  void set_clk_pin(InternalGPIOPin *pin) { this->clk_pin_ = pin; }
  void set_stb_pin(InternalGPIOPin *pin) { this->stb_pin_ = pin; }
  void set_dio_pin(GPIOPin *pin) { this->dio_pin_ = pin; }
  

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
  uint8_t humidity_ = 0; // humidity readout
  bool power_on_ = false; // is the power on?
  uint8_t fan_speed_ = 0; //0 = auto, 1 = low, 2 = med, 3 = high
  bool tank_full_ = false; // is the tank full light on?
  bool defrosting_ = false; // is the defrosting light on?
  bool already_listening_ = false; // we started listening for an update

};

}  // namespace bdh450
}  // namespace esphome
