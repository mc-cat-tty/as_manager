#include <time/timer.hpp>


namespace temporal{

Timer::Timer() :  is_started(false) {

}

void Timer::start(Tick duration) {
    if( !is_started ) {
        endtime_ = Clock::get_time() + duration;
        is_started = true;
    }
}

void Timer::stop() {
    is_started = false;
}

bool Timer::has_expired() const {
    return Clock::get_time() >= endtime_;
}


};

