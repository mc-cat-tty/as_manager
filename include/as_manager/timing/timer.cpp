#include <as_manager/timing/timer.hpp>


namespace timing{
  TimerAsync::TimerAsync() : is_started(false) {}

  void TimerAsync::start(Tick duration) {
    if( !is_started ) {
      endtime = Clock::get_time() + duration;
      is_started = true;
    }
  }

  void TimerAsync::stop() {
    is_started = false;
  }

  bool TimerAsync::has_expired() const {
    return Clock::get_time() >= endtime;
  }
};

