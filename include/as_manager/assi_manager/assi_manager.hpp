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
        enum class ManagerState {UNINIT, READY, DRIVING, FINISHED, EMERGENCY};

        static constexpr std::chrono::milliseconds STROBE_TIME = 500ms;
        static constexpr std::chrono::milliseconds BEEP_TIME = 1000ms;
        static constexpr std::chrono::milliseconds EMERGENCY_BUZZER_TIME = 10000ms;

        ManagerState currentAssiState = ManagerState::UNINIT;
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
          strobeTimer.restart(STROBE_TIME);
          hal::actions::switch_on_assi_Y();
        }

        void enableStrobeAssiB() {
          stateAssiB = AssiState::STROBING;
          strobeTimer.restart(STROBE_TIME);
          hal::actions::switch_on_assi_B();
        }

        void enableBuzzer() {
          stateBuzzer = BuzzerState::BEEPING;
          beepTimer.restart(BEEP_TIME);
          emergencyBuzzerTimer.restart(EMERGENCY_BUZZER_TIME);
          hal::actions::active_buzzer();
        }

    public:
      static AssiManager& getInstance() {
        static AssiManager instance;
        return instance;
      }

      inline void ready() {
        if (currentAssiState == ManagerState::READY) return;
        currentAssiState = ManagerState::READY;
        resetAssi();
        hal::actions::switch_on_assi_Y();
      }
      
      inline void driving() {
        if (currentAssiState == ManagerState::DRIVING) return;
        currentAssiState = ManagerState::DRIVING;
        this->resetAssi();
        this->enableStrobeAssiY();
      };
      
      inline void finished() {
        if (currentAssiState == ManagerState::FINISHED) return;
        currentAssiState = ManagerState::FINISHED;
        this->resetAssi();
        hal::actions::switch_on_assi_B();
      }
      
      inline void emergency() {
        if (currentAssiState == ManagerState::EMERGENCY) return;
        currentAssiState = ManagerState::EMERGENCY;
        this->resetAssi();
        this->enableStrobeAssiB();
        this->enableBuzzer();
      };

      void run() {
        if(stateAssiY == AssiState::STROBING && strobeTimer.has_expired()) {
          hal::set_assi_Y_state(!hal::pin::assiyPin.getValue());
          strobeTimer.restart();
        }
        else if(stateAssiB == AssiState::STROBING && strobeTimer.has_expired()) {
          hal::set_assi_B_state(!hal::pin::assibPin.getValue());
          strobeTimer.restart();
        }

        if (stateBuzzer == BuzzerState::BEEPING && emergencyBuzzerTimer.has_expired()) {
          hal::actions::disabled_buzzer();
          stateBuzzer = BuzzerState::OFF;
          emergencyBuzzerTimer.stop();
        }

        if (stateBuzzer == BuzzerState::BEEPING && beepTimer.has_expired()) {
          hal::set_buzzer_state(!hal::pin::buzzerPin.getValue());
          beepTimer.restart();
        }
    }

  };
};
