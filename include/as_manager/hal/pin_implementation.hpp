#pragma once

#include <as_manager/hal/pin.hpp>

namespace hal::pin {
    inline auto buzzerPin= KriaPin(KriaPin::Pin::BUZZER, KriaPin::Direction::OUT);
    inline auto watchdogPin= KriaPin(KriaPin::Pin::WATCHDOG, KriaPin::Direction::OUT);
    inline auto assibPin= KriaPin(KriaPin::Pin::ASSIB, KriaPin::Direction::OUT);
    inline auto assiyPin= KriaPin(KriaPin::Pin::ASSIY, KriaPin::Direction::OUT);
    inline auto ebs1Pin= KriaPin(KriaPin::Pin::EBS1, KriaPin::Direction::OUT);
    inline auto ebs2Pin= KriaPin(KriaPin::Pin::EBS2, KriaPin::Direction::OUT);
    inline auto sdcCtrlPin= KriaPin(KriaPin::Pin::SDC_CTRL, KriaPin::Direction::OUT);
    inline auto asmsPin= KriaPin(KriaPin::Pin::ASMS, KriaPin::Direction::IN);
    inline auto sdcSensPin= KriaPin(KriaPin::Pin::SDC_SENS, KriaPin::Direction::IN);
}