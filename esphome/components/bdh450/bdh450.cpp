#include "bdh450.h"
#include "sevenseg.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace bdh450 {

static const char *const TAG = "display.bdh450";
static const uint8_t BDH450_REGISTER_POWER = 0b10000000;
static const uint8_t BDH450_REGISTER_FAN_HIGH = 0b01000000;
static const uint8_t BDH450_REGISTER_FAN_MEDIUM = 0b00100000;
static const uint8_t BDH450_REGISTER_FAN_LOW = 0b00010000;
static const uint8_t BDH450_REGISTER_DEFROST = 0b00000010;
static const uint8_t BDH450_REGISTER_TANK_FULL = 0b00000001;
static const uint8_t BDH450_REGISTER_UPDATE_DISPLAY = 0xC0;


void BDH450Component::setup() {
  ESP_LOGD(TAG, "Setting up BDH-450...");

  this->clk_pin_->setup();  // INPUT
  this->dio_pin_->setup();  // INPUT
  this->stb_pin_->setup();  // INPUT

  this->clk_pin_->pin_mode(gpio::FLAG_INPUT);
  this->dio_pin_->pin_mode(gpio::FLAG_INPUT);
  this->stb_pin_->pin_mode(gpio::FLAG_INPUT);

  this->clk_pin_ = this->clk_pin_->to_isr();
  this->stb_pin_ = this->stb_pin_->to_isr();

  this->listen(true);
}

void BDH450Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BDH-450:");
  LOG_PIN("  CLK   Pin: ", this->clk_pin_);
  LOG_PIN("  DIO   Pin: ", this->dio_pin_);
  LOG_PIN("  STB   Pin: ", this->stb_pin_);

  LOG_UPDATE_INTERVAL(this);
}

void BDH450Component::loop() {
  this->process_byte_();
}



void BDH450Component::update() {  // this is called at the interval specified in the config.yaml
  
}


void IRAM_ATTR BDH450Component::strobe_active_() {
  if (!digitalRead(stb_pin_))
  {
    is_strobe_active_ = true;
  } else {
    is_strobe_active_ = false;
  }
}


void IRAM_ATTR BDH450Component::clock_read_() {
  if (!is_strobe_active_)
    return;

  bits_read_++;
  workingy_byte_ >>= 1;
  if (digitalRead(dio_pin_))
    workingy_byte_ |= 0x80;
  
  if (bits_read_ % 8 == 0)
  {
    byte_cache_[ary_idx_] = workingy_byte_;
    ary_idx_++;
    process_byte_ = true;
  }
}

void BDH450Component::listen_(bool enable_listen) {
  if (enable_listen && !already_listening_)
  {
    workingy_byte_ = 0;
    bits_read_ = 0;
    ary_idx_ = 0;

    clk_pin_->attach_interrupt(BDH450Component::clock_read_, this, gpio::INTERRUPT_RISING_EDGE);
    stb_pin_->attach_interrupt(BDH450Component::strobe_active_, this, gpio::INTERRUPT_ANY_EDGE);
    
    already_listening_ = true;
  } else if (!enable_listen) {
    
    stb_pin_->detach_interrupt();
    clk_pin_->detach_interrupt();
    already_listening_ = false;
  }
}

void BDH450Component::process_byte_()
{
  if (process_byte_)
  {
    for(int i = ary_idx_ - 1; i >= 0; i--) // read all bytes ready, usually just one
    {
      if (getting_display_) // Check if we're reading the display command parameters
      {
        switch (_wait_byte_idx_)
        {
          case 0: // LED STATUS_LIGHTS
            power_on_ = (byte_cache_[i] & BDH450_REGISTER_POWER) == BDH450_REGISTER_POWER;
            fan_speed_ = (byte_cache_[i] & BDH450_REGISTER_FAN_HIGH) == BDH450_REGISTER_FAN_HIGH?3:(byte_cache_[i] & BDH450_REGISTER_FAN_MEDIUM) == BDH450_REGISTER_FAN_MEDIUM?2:(byte_cache_[i] & BDH450_REGISTER_FAN_LOW) == BDH450_REGISTER_FAN_LOW?1:0;
            tank_full_ = (byte_cache_[i] & BDH450_REGISTER_TANK_FULL) == BDH450_REGISTER_TANK_FULL;
            defrosting_ = (byte_cache_[i] & BDH450_REGISTER_DEFROST) == BDH450_REGISTER_DEFROST;
            break;
          case 1: // empty bit? could be an issue with how we're reading data
            break;
          case 2:
            digit_ones_ = this->get_digit_(byte_cache_[i]);
            break;
          case 3: // empty bit? could be an issue with how we're reading data
            break;
          case 4:
            digit_tens_ = this->get_digit_(byte_cache_[i]);
            break;
          case 5: // empty bit? could be an issue with how we're reading data
            getting_display_ = false; // This is the last empty bit from the 0xC0
            humidity_ = digit_tens_ * 10 + digit_ones_;
            break;
        }
        _wait_byte_idx_++;
      }

      if (byte_cache_[i] == 0xC0) // Command 3 from datasheet
      {
        _wait_byte_idx_ = 0;
        getting_display_ = true; // Start interpreting the following bytes as segment display
      }

      //Serial.println(byte_cache_[i],HEX);
    }

    // processed the array, reset
    ary_idx_ = 0;    
    process_byte_ = false;
  }
}


uint8_t BDH450Component::get_digit_(uint8_t my_bits)
{
  for(int i = 0; i <= 9; i++)
    if (my_bits == SEG_DIGITS[i])
      return i;

  return 0; //uh give em something i guess?
}


}  // namespace bdh450
}  // namespace esphome
