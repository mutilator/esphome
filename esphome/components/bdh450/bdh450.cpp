#include "bdh450.h"
#include "sevenseg.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace bdh450 {

static const char *const TAG = "bdh450";
static const uint8_t BDH450_REGISTER_POWER = 0b10000000;
static const uint8_t BDH450_REGISTER_FAN_HIGH = 0b01000000;
static const uint8_t BDH450_REGISTER_FAN_MEDIUM = 0b00100000;
static const uint8_t BDH450_REGISTER_FAN_LOW = 0b00010000;
static const uint8_t BDH450_REGISTER_DEFROST = 0b00000010;
static const uint8_t BDH450_REGISTER_TANK_FULL = 0b00000001;
static const uint8_t BDH450_REGISTER_UPDATE_DISPLAY = 0xC0;




void BDH450Messenger::reset()
{
  workingy_byte = 0;
  bits_read_ = 0;
  ary_idx = 0;
}

void IRAM_ATTR BDH450Messenger::strobe_active(BDH450Messenger *msg) {

  msg->is_strobe_active_ = !msg->is_strobe_active_; //try something different? on change we just swap state i guess? this could be problematic
  /*
  if (!msg->stb_pin.digital_read())
  {
    msg->is_strobe_active_ = true;
  } else {
    msg->is_strobe_active_ = false;
  }
  */
}

void IRAM_ATTR BDH450Messenger::clock_read(BDH450Messenger *msg) {
  if (!msg->is_strobe_active_)
    return;

  msg->bits_read_++;
  msg->workingy_byte >>= 1;
  if (msg->dio_pin.digital_read())
    msg->workingy_byte |= 0x80;
  
  if (msg->bits_read_ % 8 == 0)
  {
    msg->byte_cache[msg->ary_idx] = msg->workingy_byte;
    msg->ary_idx++;
    msg->process_byte_now = true;
  }
}

void BDH450Messenger::listen(InternalGPIOPin *clk_pin, InternalGPIOPin *stb_pin, InternalGPIOPin *dio_pin, bool enable_listen) {
  
  this->stb_pin = stb_pin->to_isr();
  this->dio_pin = dio_pin->to_isr();
  this->clk_pin = clk_pin->to_isr();
  if (enable_listen && !already_listening_)
  {
    reset();

    stb_pin->attach_interrupt(BDH450Messenger::strobe_active, this, gpio::INTERRUPT_ANY_EDGE);
    clk_pin->attach_interrupt(BDH450Messenger::clock_read, this, gpio::INTERRUPT_RISING_EDGE);
    
    
    already_listening_ = true;
  } else if (!enable_listen) {
    
    stb_pin->detach_interrupt();
    clk_pin->detach_interrupt();
    already_listening_ = false;
  }
}







void BDH450Sensor::setup() {
  ESP_LOGD(TAG, "Setting up BDH-450...");

  this->clk_pin_->setup();  // INPUT
  this->dio_pin_->setup();  // INPUT
  this->stb_pin_->setup();  // INPUT

  this->clk_pin_->pin_mode(gpio::FLAG_INPUT);
  this->dio_pin_->pin_mode(gpio::FLAG_INPUT);
  this->stb_pin_->pin_mode(gpio::FLAG_INPUT);
  
  this->messenger_.listen(clk_pin_, stb_pin_, dio_pin_, true);

  ESP_LOGD(TAG, "Started listening...");
}

void BDH450Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "BDH-450:");

  LOG_UPDATE_INTERVAL(this);
}

void BDH450Sensor::update() {
  //this->messenger_.listen(clk_pin_, stb_pin_, dio_pin_, true);
  this->process_byte_();
}

bool BDH450Sensor::is_on() { return power_on_; }
bool BDH450Sensor::is_off() { return !power_on_; }

void BDH450Sensor::process_byte_()
{
  if (messenger_.process_byte_now)
  {
    for(int i = messenger_.ary_idx - 1; i >= 0; i--) // read all bytes ready, usually just one
    {
      if (getting_display_) // Check if we're reading the display command parameters
      {
        switch (_wait_byte_idx_)
        {
          case 0: // LED STATUS_LIGHTS
            power_on_ = (messenger_.byte_cache[i] & BDH450_REGISTER_POWER) == BDH450_REGISTER_POWER;

            if (this->the_fan_mode_ != nullptr)
              this->the_fan_mode_->publish_state((messenger_.byte_cache[i] & BDH450_REGISTER_FAN_HIGH) == BDH450_REGISTER_FAN_HIGH?"High":(messenger_.byte_cache[i] & BDH450_REGISTER_FAN_MEDIUM) == BDH450_REGISTER_FAN_MEDIUM?"Medium":(messenger_.byte_cache[i] & BDH450_REGISTER_FAN_LOW) == BDH450_REGISTER_FAN_LOW?"Low":"Auto");
            
            if (this->the_power_sensor_ != nullptr)
              this->the_power_sensor_->publish_state(power_on_);

            if (this->the_tank_sensor_ != nullptr)
              this->the_tank_sensor_->publish_state((messenger_.byte_cache[i] & BDH450_REGISTER_TANK_FULL) == BDH450_REGISTER_TANK_FULL);

            if (this->the_defrost_sensor_ != nullptr)
              this->the_defrost_sensor_->publish_state((messenger_.byte_cache[i] & BDH450_REGISTER_DEFROST) == BDH450_REGISTER_DEFROST);

            break;
          case 1: // empty bit? could be an issue with how we're reading data
            break;
          case 2:
            digit_ones_ = this->get_digit_(messenger_.byte_cache[i]);
            break;
          case 3: // empty bit? could be an issue with how we're reading data
            break;
          case 4:
            digit_tens_ = this->get_digit_(messenger_.byte_cache[i]);
            break;
          case 5: // empty bit? could be an issue with how we're reading data
            getting_display_ = false; // This is the last empty bit from the 0xC0
            if (this->the_humidity_sensor_ != nullptr)
              this->the_humidity_sensor_->publish_state(digit_tens_ * 10 + digit_ones_);
              this->messenger_.listen(clk_pin_, stb_pin_, dio_pin_, false);
            break;
        }
        _wait_byte_idx_++;
      }

      if (messenger_.byte_cache[i] == 0xC0) // Command 3 from datasheet
      {
        _wait_byte_idx_ = 0;
        getting_display_ = true; // Start interpreting the following bytes as segment display
      }

      
      //Serial.println(messenger_.byte_cache[i],HEX);
    }

    // processed the array, reset
    messenger_.ary_idx = 0;    
    messenger_.process_byte_now = false;
  }
}


uint8_t BDH450Sensor::get_digit_(uint8_t my_bits)
{
  for(int i = 0; i <= 9; i++)
    if (my_bits == BDH450Translation::SEG_DIGITS[i])
      return i;

  return 0; //uh give em something i guess?
}


}  // namespace bdh450
}  // namespace esphome
