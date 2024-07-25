#include <time/timer.hpp>
#include <actions/actions.hpp>

namespace watchdog {
    using namespace temporal;
    using namespace std::chrono_literals;
    using namespace hal::actions;

    class Watchdog {
        private:
            bool isToggling;
            Timer timer;
            Watchdog() : isToggling(false), timer() {}

            Watchdog(const Watchdog&) = delete;
            Watchdog& operator=(const Watchdog&) = delete;

        public:
            static Watchdog& getInstance() {
                static Watchdog instance;
                return instance;
            }

            void set_state(bool toggling) {
                isToggling = toggling;
            }

            void run() {
                if(isToggling){
                    if( !timer.it_started()){
                        toggling_watchdog();
                    }
                    if( timer.has_expired()){
                        toggling_watchdog();
                        timer.stop();
                    }
                    
                    timer.start(10s);
                }else
                    timer.stop();
            }
        };

};