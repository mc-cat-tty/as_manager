#pragma once
#include <as_manager/timing/timer.hpp>
#include <as_manager/actions/actions.hpp>
#include <as_manager/hal/pin.hpp>
#include <as_manager/hal/pin_implementation.hpp>
#include <as_manager/hal/hal.hpp>
#include <iostream>

namespace as::assi_manager {
  using namespace timing;
  using namespace std::chrono_literals;

  class AssiManager {
      private:
        enum class AssiState {STATIC, STROBING};
        enum class BuzzerState {OFF, BEEPING};

        static constexpr auto STROBE_TIME = 500ms;
        static constexpr auto BEEP_TIME = 200ms;
        static constexpr auto EMERGENCY_BUZZER_TIME = 1s;

        AssiState stateAssiY, stateAssiB;
        BuzzerState stateBuzzer;

        TimerAsync strobeTimer, beepTimer, emergencyBuzzerTimer;
        
        AssiManager() :
          stateAssiY(AssiState::STATIC),
          stateAssiB(AssiState::STATIC),
          stateBuzzer(BuzzerState::OFF) {}

        AssiManager(const AssiManager&) = delete;
        AssiManager(AssiManager&&) = delete;
        AssiManager& operator=(const AssiManager&) = delete;
        
        inline void resetAssi() {
          stateAssiY = stateAssiB = AssiState::STATIC;
          hal::actions::switch_off_assi_B();
          hal::actions::switch_off_assi_Y();
        }

        void enableStrobeAssiY() {
          stateAssiY = AssiState::STROBING;
          strobeTimer.start(STROBE_TIME);
          hal::actions::switch_on_assi_Y();
        }

        void enableStrobeAssiB() {
          stateAssiB = AssiState::STROBING;
          strobeTimer.start(STROBE_TIME);
          hal::actions::switch_on_assi_B();
        }

        void enableBuzzer() {
          stateBuzzer = BuzzerState::BEEPING;
          beepTimer.start(BEEP_TIME);
          emergencyBuzzerTimer.start(EMERGENCY_BUZZER_TIME);
          hal::actions::active_buzzer();
        }

    public:
      static AssiManager& getInstance() {
        static AssiManager instance;
        return instance;
      }

      inline void ready() {
        resetAssi();
        hal::actions::switch_on_assi_Y();
      }
      
      inline void driving() {
        this->resetAssi();
        this->enableStrobeAssiY();
      };
      
      inline void finished() {
        this->resetAssi();
        hal::actions::switch_on_assi_B();
      }
      
      inline void emergency() {
        this->resetAssi();
        this->enableStrobeAssiB();
        this->enableBuzzer();
      };

      void run() {
        if(stateAssiY == AssiState::STROBING && strobeTimer.has_expired()) {
          strobeTimer.stop();
          hal::set_assi_Y_state(!hal::pin::assiyPin.getValue());
          strobeTimer.start(STROBE_TIME);
        }
        else if(stateAssiB == AssiState::STROBING && strobeTimer.has_expired()) {
          strobeTimer.stop();
          hal::set_assi_B_state(!hal::pin::assibPin.getValue());
          strobeTimer.start(STROBE_TIME);
        }

        if (emergencyBuzzerTimer.has_expired()) {
          hal::actions::disabled_buzzer();
          stateBuzzer = BuzzerState::OFF;
        }

        if (stateBuzzer == BuzzerState::BEEPING && beepTimer.has_expired()) {
          beepTimer.stop();
          hal::set_buzzer_state(!hal::pin::buzzerPin.getValue());
          beepTimer.start(BEEP_TIME);
        }
    }

  };
};
