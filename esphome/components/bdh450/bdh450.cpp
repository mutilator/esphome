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
  working_byte = 0;
  bits_read_ = 0;
  
  reset_stages();
}


void IRAM_ATTR BDH450Messenger::strobe_active(BDH450Messenger *msg) {
  if (!msg->stb_pin.digital_read())
  {
    msg->is_strobe_active_ = true;
  } else {
    msg->is_strobe_active_ = false;
    msg->getting_display_ = false;
    msg->bits_read_ = 0;
  }
}

void IRAM_ATTR BDH450Messenger::clock_read(BDH450Messenger *msg) {
  if (!msg->is_strobe_active_)
    return;

  msg->bits_read_++;
  msg->working_byte >>= 1;
  if (msg->dio_pin.digital_read())
    msg->working_byte |= 0x80;
  
  
  if (msg->bits_read_ % 8 == 0)
  {
    msg->process_byte_(msg->working_byte);
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


void IRAM_ATTR BDH450Messenger::process_byte_(uint8_t worked_byte)
{
  if (getting_display_) // Check if we're reading the display command parameters
    {
      switch (_wait_byte_idx_)
      {
        case 0: // LED STATUS_LIGHTS
          power_on_ = (worked_byte & BDH450_REGISTER_POWER) == BDH450_REGISTER_POWER;
          fan_speed_ = (worked_byte & BDH450_REGISTER_FAN_HIGH) == BDH450_REGISTER_FAN_HIGH?3:(worked_byte & BDH450_REGISTER_FAN_MEDIUM) == BDH450_REGISTER_FAN_MEDIUM?2:(worked_byte & BDH450_REGISTER_FAN_LOW) == BDH450_REGISTER_FAN_LOW?1:0;
          tank_full_ = (worked_byte & BDH450_REGISTER_TANK_FULL) == BDH450_REGISTER_TANK_FULL;
          defrosting_ = (worked_byte & BDH450_REGISTER_DEFROST) == BDH450_REGISTER_DEFROST;
          stage_statuses_[_wait_byte_idx_] = true;
          break;
        case 1: // empty bit? could be an issue with how we're reading data
          if (worked_byte == 0)
            stage_statuses_[_wait_byte_idx_] = true;
          break;
        case 2:
          
          digit_ones_ = BDH450Sensor::get_digit(worked_byte);
          if (digit_ones_ < 10)
            stage_statuses_[_wait_byte_idx_] = true;
          break;
        case 3: // empty bit? could be an issue with how we're reading data
          if (worked_byte == 0)
            stage_statuses_[_wait_byte_idx_] = true;
          break;
        case 4:
          digit_tens_ = BDH450Sensor::get_digit(worked_byte);
          if (digit_tens_ < 10)
            stage_statuses_[_wait_byte_idx_] = true;
          break;
        case 5: // empty bit? could be an issue with how we're reading data
          if (worked_byte == 0)
            stage_statuses_[_wait_byte_idx_] = true;

          getting_display_ = false; // This is the last empty bit from the 0xC0

          bool success = true;
          //make sure it was all a success
          for(int i = 0; i < 6; i++) { if (!stage_statuses_[i]) { success = false; } }

          if (success)
          {
            power_on = power_on_;
            fan_speed = fan_speed_;
            tank_full = tank_full_;
            defrosting = defrosting_;

            if (power_on)
            {
              // only update if power is on and it's reading a sensible range
              // easier to ignore typical outliers than to create a way to make sure it's not reading incorrectly
              if ((digit_tens_ * 10 + digit_ones_) > 10 && (digit_tens_ * 10 + digit_ones_) < 90) 
              {
                humidity = digit_tens_ * 10 + digit_ones_;
              }
            } else {
              humidity = 0; //power is off, no readings
            }
          }
          break;
      }
      _wait_byte_idx_++;
    }

    if (worked_byte == 0xC0) // Command 3 from datasheet
    {
      _wait_byte_idx_ = 0;
      getting_display_ = true; // Start interpreting the following bytes as segment display
    }
    last_worked_byte = worked_byte;
    //Serial.println(worked_byte,HEX);
}






void BDH450Sensor::setup() {
  ESP_LOGD(TAG, "Setting up BDH-450...");

  this->clk_pin_->setup();  // INPUT
  this->dio_pin_->setup();  // INPUT
  this->stb_pin_->setup();  // INPUT

  this->clk_pin_->pin_mode(gpio::FLAG_INPUT);
  this->dio_pin_->pin_mode(gpio::FLAG_INPUT);
  this->stb_pin_->pin_mode(gpio::FLAG_INPUT);
  
  // set defaults
  if (this->the_fan_mode_ != nullptr)
    this->the_fan_mode_->publish_state("Auto");
  
  if (this->the_power_sensor_ != nullptr)
    this->the_power_sensor_->publish_state(false);

  if (this->the_tank_sensor_ != nullptr)
    this->the_tank_sensor_->publish_state(false);

  if (this->the_defrost_sensor_ != nullptr)
    this->the_defrost_sensor_->publish_state(false);

  if (this->the_humidity_sensor_ != nullptr)
    this->the_humidity_sensor_->publish_state(0);


  this->messenger_.listen(clk_pin_, stb_pin_, dio_pin_, true);

  ESP_LOGD(TAG, "Started listening...");
}

void BDH450Sensor::dump_config() {
  ESP_LOGCONFIG(TAG, "BDH-450:");

  LOG_UPDATE_INTERVAL(this);
}

void BDH450Sensor::loop() {
}

void BDH450Sensor::update() {
  if (this->the_fan_mode_ != nullptr)
    this->the_fan_mode_->publish_state(messenger_.fan_speed==3?"High":messenger_.fan_speed==2?"Medium":messenger_.fan_speed==1?"Low":"Auto");
  
  if (this->the_power_sensor_ != nullptr)
    this->the_power_sensor_->publish_state(messenger_.power_on);

  if (this->the_tank_sensor_ != nullptr)
    this->the_tank_sensor_->publish_state(messenger_.tank_full);

  if (this->the_defrost_sensor_ != nullptr)
    this->the_defrost_sensor_->publish_state(messenger_.defrosting);
  
  if (this->the_humidity_sensor_ != nullptr)
    this->the_humidity_sensor_->publish_state(messenger_.humidity);
}

bool BDH450Sensor::is_on() { return messenger_.power_on; }
bool BDH450Sensor::is_off() { return !messenger_.power_on; }




uint8_t BDH450Sensor::get_digit(uint8_t my_bits)
{
  for(int i = 0; i <= 9; i++)
    if (my_bits == BDH450Translation::SEG_DIGITS[i])
      return i;

  return 0; //uh give em something i guess?
}


}  // namespace bdh450
}  // namespace esphome
