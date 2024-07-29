#include <as_manager/temporal/timer.hpp>
#include <as_manager/hal/hal.hpp>
#include <iostream>

namespace watchdog {
    using namespace temporal;
    using namespace std::chrono_literals;

    class Watchdog {
        private:
            bool isToggling, pinState;
            Timer timer;
            Watchdog() : isToggling(false), pinState(false), timer() {}

            Watchdog(const Watchdog&) = delete;
            Watchdog(Watchdog&&) = delete;
            Watchdog& operator=(const Watchdog&) = delete;

            inline void togglePinState(){
                pinState ^= true;
            }

        public:
            static Watchdog& getInstance() {
                static Watchdog instance;
                return instance;
            }

            void set_toggling() {
                isToggling = true;
                togglePinState();
                hal::write_watchdog_state(pinState);
                //std::cout<<"[WATCHDOG] set pin to: "<<pinState<<std::endl;
            }

            void stop_toggling() {
                isToggling = false;
            }


            void run() {
                if(!isToggling){
                    timer.stop();
                    return;
                }
                timer.start(100ms);
                if( timer.has_expired()){
                    togglePinState();
                    hal::write_watchdog_state(pinState);
                    //std::cout<<"[WATCHDOG] set pin to: "<<pinState<<std::endl;
                    timer.stop();
                }
               // //std::cout<<"[WATCHDOG] idle "<<std::endl;
                    
            }
        };

};