#pragma once

#include <time/interfaces.hpp>
#include <time/clock.hpp>

namespace temporal {


class Timer
{
public:

    Timer();

    bool has_expired() const;

    void start(temporal::Tick duration);

    void stop();

private:
    bool is_started;

    temporal::Tick endtime_;
    temporal::Tick duration;

};
};