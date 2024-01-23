#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"


#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>


namespace esphome {
namespace bdh450 {

class BDH450Messenger {

  public:
    static void clock_read(BDH450Messenger *msg);
    static void strobe_active(BDH450Messenger *msg);
    void reset();

    void listen(InternalGPIOPin *clk_pin, InternalGPIOPin *stb_pin, InternalGPIOPin *dio_pin, bool enable_listen);

    ISRInternalGPIOPin stb_pin;
    ISRInternalGPIOPin dio_pin;
    ISRInternalGPIOPin clk_pin;
    volatile bool process_byte_now = false; // Whether the main loop should process the data read
    volatile uint8_t ary_idx = 0; // where we're writing to the array
    volatile uint8_t byte_cache[5]; // small array in case we get data faster than we're processing it
    volatile uint8_t workingy_byte = 0; // Last data byte read from the serial
    

  protected:
    
    volatile bool already_listening_ = false; // we started listening for an update
    volatile bool is_strobe_active_ = false; // If stroibe is active we're reading data
    volatile int bits_read_ = 0; // how many bits read so far, we kick it at 8 no matter what
    
};

class BDH450Sensor : public PollingComponent {
 public:
  void setup() override;
  void dump_config() override;

  void set_clk_pin(InternalGPIOPin *pin) { this->clk_pin_ = pin; }
  void set_stb_pin(InternalGPIOPin *pin) { this->stb_pin_ = pin; }
  void set_dio_pin(InternalGPIOPin *pin) { this->dio_pin_ = pin; }
  void set_power_sensor(binary_sensor::BinarySensor *sensor) { the_power_sensor_ = sensor; }
  void set_tank_sensor(binary_sensor::BinarySensor *sensor) { the_tank_sensor_ = sensor; }
  void set_defrost_sensor(binary_sensor::BinarySensor *sensor) { the_defrost_sensor_ = sensor; }
  void set_fan_mode_sensor(text_sensor::TextSensor *sensor) { the_fan_mode_ = sensor; }
  void set_humidity_sensor(sensor::Sensor *sensor) { the_humidity_sensor_ = sensor; }
  
  bool is_on();
  bool is_off();

  void update() override;

 protected:
  uint8_t get_digit_(uint8_t my_bits);

  void process_byte_();



  InternalGPIOPin *clk_pin_;
  InternalGPIOPin *stb_pin_;
  InternalGPIOPin *dio_pin_;

  binary_sensor::BinarySensor *the_power_sensor_{nullptr};
  binary_sensor::BinarySensor *the_tank_sensor_{nullptr};
  binary_sensor::BinarySensor *the_defrost_sensor_{nullptr};
  text_sensor::TextSensor *the_fan_mode_{nullptr};
  sensor::Sensor *the_humidity_sensor_{nullptr};


  BDH450Messenger messenger_; // pass messages from the interrupt

  uint8_t _wait_byte_idx_ = 0; // which byte of the 6 are we processing
  bool getting_display_ = false; // if we're parsing the 0xC0 command that pushes data to the display

  uint8_t digit_ones_ = 0; // temp hold ones digit
  uint8_t digit_tens_ = 0; // temp hold tens digit
  bool power_on_ = false; // is the power on?
  

};

}  // namespace bdh450
}  // namespace esphome
