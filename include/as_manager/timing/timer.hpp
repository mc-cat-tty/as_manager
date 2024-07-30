#pragma once

#include <as_manager/timing/clock.hpp>

namespace timing {


class Timer
{
public:

    Timer();

    bool has_expired() const;

    void start(timing::Tick duration);

    void stop();

private:
    bool is_started;

    timing::Tick endtime_;
    timing::Tick duration;

};
};