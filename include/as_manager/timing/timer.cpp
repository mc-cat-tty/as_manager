#include <as_manager/timing/timer.hpp>


namespace timing{
  TimerAsync::TimerAsync() : is_started(false) {}

  void TimerAsync::start(milliseconds duration) {
    if( !is_started ) {
      endtime = Clock::get_time<milliseconds>() + duration;
      is_started = true;
    }
  }

  void TimerAsync::restart(milliseconds duration) {
    stop();
    start(duration);
  }

  void TimerAsync::restart() {
    stop();
    start(this->duration);
  }

  void TimerAsync::stop() {
    is_started = false;
  }

  bool TimerAsync::has_expired() const {
    return Clock::get_time<milliseconds>() >= endtime;
  }
};

