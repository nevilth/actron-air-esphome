#include "actron_air_keypad.h"

namespace esphome {
  namespace actron_air_esphome {

    static const char *const TAG = "actron_air_esphome";

    // Mapping from BinarySensorId to LedIndex with inversion flag
    struct BinarySensorMapping {
      LedIndex led_index;
      bool invert;
    };

    // Static mapping table - order must match BinarySensorId enum
    static constexpr std::array<BinarySensorMapping, BINARY_SENSOR_COUNT>
        BINARY_SENSOR_MAPPINGS = {{
            {LedIndex::FAN_CONT, false},
            {LedIndex::FAN_HIGH, false},
            {LedIndex::FAN_MID, false},
            {LedIndex::FAN_LOW, false},
            {LedIndex::COOL, false},
            {LedIndex::AUTO_MODE, false},
            {LedIndex::HEAT, false},
            {LedIndex::RUN, false},
            {LedIndex::TIMER, false},
            {LedIndex::SETPOINT, false},
        }};

    // Mapping from ZoneSensorId to LedIndex with inversion flag
    struct ZoneSensorMapping {
      LedIndex led_index;
      bool invert;
    };

    // Static mapping table - order must match ZoneSensorId enum
    static constexpr std::array<ZoneSensorMapping, ZONE_SENSOR_COUNT>
        ZONE_SENSOR_MAPPINGS = {{
            {LedIndex::ZONE_1, false},
            {LedIndex::ZONE_2, false},
            {LedIndex::ZONE_3, false},
            {LedIndex::ZONE_4, false},
            {LedIndex::ZONE_5, true},  // Zones 5-8 have inverted logic
            {LedIndex::ZONE_6, true},
            {LedIndex::ZONE_7, true},
            {LedIndex::ZONE_8, true},
        }};

    // Decode 7-segment pattern (GFEDCBA) to character
    //    AAA
    //   F   B
    //    GGG
    //   E   C
    //    DDD
    char ActronAirKeypad::decode_digit(uint8_t pattern) {
      switch (pattern) {
      case 0x3F:
        return '0'; // ABCDEF
      case 0x06:
        return '1'; // BC
      case 0x5B:
        return '2'; // ABDEG
      case 0x4F:
        return '3'; // ABCDG
      case 0x66:
        return '4'; // BCFG
      case 0x6D:
        return '5'; // ACDFG
      case 0x07:
        return '7'; // ABC
      case 0x7C:
        return '6'; // CDEFG (no A)
      case 0x7F:
        return '8'; // ABCDEFG
      case 0x67:
        return '9'; // ABCFG (no D)

      case 0x00:
        return ' '; // blank
      case 0x40:
        return '-'; // G (dash)

      case 0x79:
        return 'E'; // ADEFG
      case 0x5F:
        return 'S'; // ABDEFG
      case 0x73:
        return 'P'; // ABEFG

      default:
        ESP_LOGE(TAG, "Unknown 7-segment pattern: 0x%02X", pattern);
        return '?';
      }
    }

    std::string ActronAirKeypad::get_display_string() const {

      using L = LedIndex;

      // Extract segment bits for each digit (GFEDCBA pattern)
      auto extract = [this](L a, L b, L c, L d, L e, L f, L g) {
        return static_cast<uint8_t>((get_pulse(g) << 6) | (get_pulse(f) << 5) |
                                    (get_pulse(e) << 4) | (get_pulse(d) << 3) |
                                    (get_pulse(c) << 2) | (get_pulse(b) << 1) |
                                    get_pulse(a));
      };

      uint8_t d1 = extract(L::DIGIT1_A, L::DIGIT1_B, L::DIGIT1_C, L::DIGIT1_D,
                          L::DIGIT1_E, L::DIGIT1_F, L::DIGIT1_G);
      uint8_t d2 = extract(L::DIGIT2_A, L::DIGIT2_B, L::DIGIT2_C, L::DIGIT2_D,
                          L::DIGIT2_E, L::DIGIT2_F, L::DIGIT2_G);
      uint8_t d3 = extract(L::DIGIT3_A, L::DIGIT3_B, L::DIGIT3_C, L::DIGIT3_D,
                          L::DIGIT3_E, L::DIGIT3_F, L::DIGIT3_G);

      char c1 = decode_digit(d1);
      char c2 = decode_digit(d2);
      char c3 = decode_digit(d3);

      std::string result;
      result += c1;
      result += c2;   
      if (get_pulse(L::DP)) {
        result += '.';
      }
      result += c3;
      return result;

    }    
    
    float ActronAirKeypad::get_setpoint_value(std::string display_string) const {

      if (display_string.length() < 3) {
        return -1.0f; // Invalid display length
      }
      
      //convert string to float
      
      float display_value = 0;
      const auto format = std::chars_format::general;
      const auto res = std::from_chars(display_string.data(), display_string.data() + display_string.size(), display_value, format);
      
      if (res.ec == std::errc()) {
        return display_value;
      } else {
        ESP_LOGE(TAG, "Failed to parse display value: %.*s", static_cast<int>(display_string.size()), display_string.data());
        return -1.0f; // Parsing error  
      }

    }

    void IRAM_ATTR ActronAirKeypad::handle_interrupt(ActronAirKeypad *arg) {
      ActronAirKeypad *keypad = static_cast<ActronAirKeypad *>(arg);
      uint64_t now_us = esp_timer_get_time();
      
      if (keypad->last_intr_us_ == 0) {
        // First interrupt - just initialize timestamp
        keypad->last_intr_us_ = now_us;
        return;
      }

      // Calculate time since last pulse and save timestamp for next calculation
      uint64_t delta_us = now_us - keypad->last_intr_us_;
      
      if (delta_us < DEBOUNCE_US) {
        // We are within the debounce period - ignore this pulse
        return;
      }
      
      keypad->last_intr_us_ = now_us;
      
      uint64_t frame_age_us = 0;
      // Calculate age of current frame if we have seen a start condition - this is used to detect if we have timed out waiting for the rest of the frame
      if(keypad->frame_start_us_ != 0) {
        frame_age_us = now_us - keypad->frame_start_us_;
      }
      
      uint8_t idx = keypad->local_num_pulses_;

      //buffer start is first pulse after an extended period of inactivity (start condition = 90mSec without pulses)
      // This indicates the start of a new frame - publish last frame (if valid), reset state and start collecting pulses
      if (delta_us >= START_CONDITION_US) {

        if(keypad->in_frame_) {
          // We have a full frame - publish it
          if (idx == NPULSE) {
            keypad->pulse_bits_.store(keypad->local_pulse_bits_, std::memory_order_relaxed);
            keypad->last_error_code_.store(ERROR_NONE, std::memory_order_relaxed);
          }
          else if (idx > NPULSE) {
            // We have more pulses than expected
            keypad->last_error_code_.store(ERROR_FRAME_OVERFLOW, std::memory_order_relaxed);
            keypad->error_count_.fetch_add(1, std::memory_order_relaxed);
          } else {
            // Not a full frame 
            keypad->last_error_code_.store(ERROR_INCOMPLETE_FRAME, std::memory_order_relaxed);
            keypad->error_count_.fetch_add(1, std::memory_order_relaxed);
          }   
              
          // Save state and signal main loop to process frame or error
          keypad->num_pulses_.store(idx, std::memory_order_release);
          keypad->signal_main_loop_.store(true, std::memory_order_release);
          keypad->enable_loop_soon_any_context();
        } 
      
        // Reset staging buffer for new frame
        keypad->local_pulse_bits_ = 0;
        keypad->local_num_pulses_ = 0;
        keypad->frame_start_us_ = now_us;
        keypad->in_frame_ = true;
      
        return;
      } 
      
      if (!keypad->in_frame_) {
        // We haven't seen a start condition yet - ignore any pulses
        return;
      }

      //check error conditions before processing pulse - if any of these are true then we know the current frame is invalid and we can abort early without doing any more work
      //if a bit boundary time has elapsed then error - this means we missed some pulses and the current frame is invalid
      if (delta_us > BIT_BOUNDARY_US) {
        keypad->in_frame_ = false;
        keypad->num_pulses_.store(idx, std::memory_order_release);
        keypad->last_error_code_.store(ERROR_BIT_BOUNDARY, std::memory_order_relaxed);
        keypad->error_count_.fetch_add(1, std::memory_order_relaxed);
        keypad->signal_main_loop_.store(true, std::memory_order_release);
        keypad->enable_loop_soon_any_context();
        return;
      }

        //if a frame boundary time has elapsed then error - this means we missed some pulses and the current frame is invalid
      if (frame_age_us > FRAME_TIMEOUT_US) {
        keypad->in_frame_ = false;
        keypad->num_pulses_.store(idx, std::memory_order_release);
        keypad->last_error_code_.store(ERROR_FRAME_TIMEOUT, std::memory_order_relaxed);
        keypad->error_count_.fetch_add(1, std::memory_order_relaxed);
        keypad->signal_main_loop_.store(true, std::memory_order_release);
        keypad->enable_loop_soon_any_context();
        return;
      }

      // Set bit in staging buffer if pulse is short (logic 1)
      if (delta_us < PULSE_THRESHOLD_US && idx < NPULSE) {
        keypad->local_pulse_bits_ |= (1ULL << idx);
      }
      
      keypad->local_num_pulses_ =idx + 1;

    }

    void ActronAirKeypad::setup() {
      if (!pin_) {
        ESP_LOGE(TAG, "Pin not configured");

        return;
      }

      ESP_LOGD(TAG, "Setting up on pin");
      pin_->setup();

      auto *internal_pin = static_cast<InternalGPIOPin *>(pin_);
      ESP_LOGD(TAG, "Setting up interrupt");
      internal_pin->attach_interrupt(handle_interrupt, this,
                                    gpio::INTERRUPT_FALLING_EDGE);
    }

    void ActronAirKeypad::loop() {

      //if we haven't been signaled by the ISR  then nothing to do
      if (!this->signal_main_loop_.load(std::memory_order_acquire)) {
        this->disable_loop();
        return;
      }

      // Reset signal for main loop
      this->signal_main_loop_.store(false, std::memory_order_release);

      uint8_t num_pulses = num_pulses_.load(std::memory_order_acquire);
      
      //If we have a full frame then process.
      if (num_pulses && num_pulses == NPULSE){
          // No InterruptLock needed - single atomic load
          uint64_t bits = pulse_bits_.load(std::memory_order_acquire);

          if (bits != this->pulses_) {
            this->pulses_ = bits;
            this->has_new_data_ = true;
            this->status_count_++;

            if (status_count_sensor_) { //update status count when data changes
            status_count_sensor_->publish_state(status_count_);
            }        
          }  
      } else {
        uint8_t error_code = this->last_error_code_.load(std::memory_order_relaxed);
        ESP_LOGD(TAG, "%u bits received (or data error code: %u)", num_pulses, error_code);
        
        //publish error count  even if we don't have a full frame - this way we can track errors and see if we are getting partial frames which can be useful for troubleshooting
        if (error_count_sensor_) {
          error_count_sensor_->publish_state(
          error_count_.load(std::memory_order_relaxed));
        }
      }
      
      if (!this->has_new_data_) {
        return;
      }

      this->has_new_data_ = false;
      ESP_LOGD(TAG, "New data available");

      if (bit_string_) {
        std::string text;
        text.reserve(NPULSE);
        for (std::size_t i = 0; i < NPULSE; ++i) {
          text += ((pulses_ >> i) & 1) ? '1' : '0';
        }
        ESP_LOGD(TAG, "bit string: %s", text.c_str());
        bit_string_->publish_state(text);
      }

      std::string display_string = get_display_string();
      if (lcd_string_) {
        lcd_string_->publish_state(display_string);
      }
      if (setpoint_temp_) {
        if (get_pulse(LedIndex::SETPOINT)) {
          // If the setpoint LED is on, we assume the display is showing the setpoint temperature
          setpoint_temp_->publish_state(get_setpoint_value(display_string));
        } 
      }

      for (std::size_t i = 0; i < BINARY_SENSOR_COUNT; ++i) {
        if (binary_sensors_[i]) {
          const auto &mapping = BINARY_SENSOR_MAPPINGS[i];
          bool state = get_pulse(mapping.led_index);
          binary_sensors_[i]->publish_state(mapping.invert ? !state : state);
        }
      }

      for (std::size_t i = 0; i < ZONE_SENSOR_COUNT; ++i) {
        if (zone_sensors_[i]) {
          const auto &mapping = ZONE_SENSOR_MAPPINGS[i];
          bool state = get_pulse(mapping.led_index);
          state = mapping.invert ? !state : state;
          state = state && get_pulse(LedIndex::SETPOINT); // if setpoint LED is on then we are showing the setpoint temp and not the current temp, in this case we want to show all zones as off
          zone_sensors_[i]->publish_state(state);
        }
      }

      // set the virtual inside binary sensor.
     if(inside_) {
        if (!get_pulse(LedIndex::SETPOINT) && get_pulse(LedIndex::ZONE_8)) {
          // If the setpoint LED is off and the zone 8 LED is on, we can be pretty confident that we are showing the current temperature and that we are inside, so we can publish the inside binary sensor as true
          inside_->publish_state(true);
       } else {
             inside_->publish_state(false);
        }
      }
    }

    void ActronAirKeypad::dump_config() {
      ESP_LOGCONFIG(TAG, "Actron Air ESPHome:");
      LOG_PIN("  Pin: ", this->pin_);
    }

  } // namespace actron_air_esphome
} // namespace esphome
