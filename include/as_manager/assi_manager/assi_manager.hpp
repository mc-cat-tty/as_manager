#pragma once
#include <as_manager/temporal/timer.hpp>
#include <as_manager/actions/actions.hpp>
#include <iostream>

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
                enalbeAssiB=false;
                strobeAssiB=false;
                strobeAssiY=false;
            }

            void enableAssiB() {
                enalbeAssiY=false;
                enalbeAssiB=true;
                strobeAssiB=false;
                strobeAssiY=false;
            }

            void enableStrobeAssiY() {
                enalbeAssiY=false;
                enalbeAssiB=false;
                strobeAssiB=false;
                strobeAssiY=true;
                assiYTimer.start(500ms);
                hal::actions::switch_on_assi_Y();
                assiYState = !assiYState;
                //std::cout<<"[ASSI_MANAGER][STROBE] enableAssiY"<<std::endl;
            }

            void enableStrobeAssiB() {
                enalbeAssiY=false;
                enalbeAssiB=false;
                strobeAssiB=true;
                strobeAssiY=false;
                assiBTimer.start(200ms);
                hal::actions::switch_on_assi_B();
                assiBState = !assiBState;
                //std::cout<<"[ASSI_MANAGER][STROBE] enableAssiB"<<std::endl;
            }

            void enableBuzzer() {
                enabledBuzzer=true;
                buzzerTimer.start(500ms);
                hal::actions::active_buzzer();
                buzzerState = !buzzerState;
                //std::cout<<"[ASSI_MANAGER][BUZZER] enalbe"<<std::endl;
            }


            void run() {
                if (enalbeAssiY) {
                    hal::actions::switch_on_assi_Y();
                    //std::cout<<"[ASSI_MANAGER] enableAssiY"<<std::endl;
                }else if(enalbeAssiB){
                    hal::actions::switch_on_assi_B();
                    //std::cout<<"[ASSI_MANAGER] enableAssiB"<<std::endl;
                }else if(strobeAssiY && assiYTimer.has_expired()) {
                    assiYTimer.stop();
                    if (assiYState) {
                        hal::actions::switch_off_assi_Y();
                        //std::cout<<"[ASSI_MANAGER][STROBE] disableAssiY"<<std::endl;
                    } else {
                        hal::actions::switch_on_assi_Y();
                        //std::cout<<"[ASSI_MANAGER][STROBE] enalbeAssiY"<<std::endl;
                    }
                    assiYState = !assiYState;
                    assiYTimer.start(500ms);
                }else if(strobeAssiB && assiBTimer.has_expired()) {
                    assiBTimer.stop();
                    if (assiBState) {
                        hal::actions::switch_off_assi_B();
                        //std::cout<<"[ASSI_MANAGER][STROBE] disableAssiB"<<std::endl;
                    } else {
                        hal::actions::switch_on_assi_B();
                        //std::cout<<"[ASSI_MANAGER][STROBE] enalbeAssiB"<<std::endl;
                    }
                    assiBState = !assiBState;
                    assiBTimer.start(200ms);
                }

                if (enabledBuzzer && buzzerTimer.has_expired()) {
                    buzzerTimer.stop();
                    if (buzzerState) {
                        hal::actions::disabled_buzzer();
                        //std::cout<<"[ASSI_MANAGER][BUZZER] disable"<<std::endl;
                    } else {
                        hal::actions::active_buzzer();
                        //std::cout<<"[ASSI_MANAGER][BUZZER] enalbe"<<std::endl;
                    }
                    buzzerState = !buzzerState;
                    buzzerTimer.start(500ms);

                    buzzerDurationCounter += 500; // Incrementa il contatore di durata di 500 ms
                    if (buzzerDurationCounter >= 9000) { // Se la durata totale ha raggiunto i 10 secondi
                        enabledBuzzer = false;
                        buzzerState = false;
                        buzzerDurationCounter =0 ;
                        //std::cout<<"[ASSI_MANAGER][BUZZER] stopped"<<std::endl;
                        hal::actions::disabled_buzzer();
                    }
                }
                ////std::cout<<"[ASSI_MANAGER] IDLE"<<std::endl;
            }
        };
};
