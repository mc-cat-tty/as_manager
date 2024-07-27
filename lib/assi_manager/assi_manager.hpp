#pragma once
#include <temporal/timer.hpp>
#include <actions/actions.hpp>

namespace as::assi_manager {

    using namespace temporal;
    using namespace std::chrono_literals;

    class AssiManager {
         private:
            bool enalbeAssiY,enalbeAssiB, strobeAssiY, strobeAssiB, enabledBuzzer, buzzerState, assiYState, assiBState;
            Timer timer,buzzerTimer,assiYTimer,assiBTimer;
            int buzzerDurationCounter;
            AssiManager() : enalbeAssiY(false),enalbeAssiB(false), strobeAssiY(false), strobeAssiB(false), enabledBuzzer(false), buzzerState(false), assiYState(false), assiBState(false), buzzerTimer(), timer(), assiYTimer(), assiBTimer(), buzzerDurationCounter(0) {}

            AssiManager(const AssiManager&) = delete;
            AssiManager& operator=(const AssiManager&) = delete;

        public:
            static AssiManager& getInstance() {
                static AssiManager instance;
                return instance;
            }

            void enableAssiY() {
                enalbeAssiY=true;
            }

            void enableAssiB() {
                enalbeAssiY=true;
            }

            void enableStrobeAssiY() {
                strobeAssiY=true;
                assiYTimer.start(500ms);
                hal::actions::switch_on_assi_Y();
                assiYState = !assiYState;
            }

            void enableStrobeAssiB() {
                strobeAssiB=true;
                assiBTimer.start(200ms);
                hal::actions::switch_on_assi_B();
                assiBState = !assiBState;
            }

            void enableBuzzer() {
                enabledBuzzer=true;
                buzzerTimer.start(500ms);
                hal::actions::active_buzzer();
                buzzerState = !buzzerState;
            }


            void run() {
                if (enalbeAssiY) {
                    hal::actions::switch_on_assi_Y();
                }else if(enalbeAssiB){
                    hal::actions::switch_on_assi_B();
                }else if(strobeAssiY && assiYTimer.has_expired()) {
                    assiYTimer.stop();
                    if (assiYState) {
                        hal::actions::switch_off_assi_Y();
                    } else {
                        hal::actions::switch_on_assi_Y();
                    }
                    assiYState = !assiYState;
                    assiYTimer.start(500ms);
                }else if(strobeAssiB && assiBTimer.has_expired()) {
                    assiBTimer.stop();
                    if (assiBState) {
                        hal::actions::switch_off_assi_B();
                    } else {
                        hal::actions::switch_on_assi_B();
                    }
                    assiBState = !assiYState;
                    assiBTimer.start(200ms);
                }

                if (enabledBuzzer && buzzerTimer.has_expired()) {
                    buzzerTimer.stop();
                    if (buzzerState) {
                        hal::actions::disabled_buzzer();
                    } else {
                        hal::actions::active_buzzer();
                    }
                    buzzerState = !buzzerState;
                    buzzerTimer.start(500ms);

                    buzzerDurationCounter += 500; // Incrementa il contatore di durata di 500 ms
                    if (buzzerDurationCounter >= 10000) { // Se la durata totale ha raggiunto i 10 secondi
                        enabledBuzzer = false;
                        buzzerState = false;
                        buzzerDurationCounter =0 ;
                        hal::actions::disabled_buzzer();
                    }
                }
            }
        };
};
