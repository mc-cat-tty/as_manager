#include <temporal/timer.hpp>
#include <actions/actions.hpp>

namespace watchdog {
    using namespace temporal;
    using namespace std::chrono_literals;

    class Watchdog {
        private:
            bool isToggling, pinState;
            Timer timer;
            Watchdog() : isToggling(false), pinState(false), timer() {}

            Watchdog(const Watchdog&) = delete;
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
            }

            void stop_togglinf() {
                isToggling = false;
            }


            void run() {
                if(!isToggling){
                    timer.stop();
                    return;
                }
                timer.start(10s);
                if( timer.has_expired()){
                    togglePinState();
                    hal::write_watchdog_state(pinState);
                    timer.stop();
                }
                    
            }
        };

};