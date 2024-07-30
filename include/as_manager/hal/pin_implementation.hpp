#pragma once

#include <as_manager/hal/pin.hpp>

namespace hal::pin {
    auto buzzerPin= KriaPin(KriaPin::Pin::BUZZER, KriaPin::Direction::OUT);
    auto watchdogPin= KriaPin(KriaPin::Pin::WATCHDOG, KriaPin::Direction::OUT);
    auto assibPin= KriaPin(KriaPin::Pin::ASSIB, KriaPin::Direction::OUT);
    auto assiyPin= KriaPin(KriaPin::Pin::ASSIY, KriaPin::Direction::OUT);
    auto ebs1Pin= KriaPin(KriaPin::Pin::EBS1, KriaPin::Direction::OUT);
    auto ebs2Pin= KriaPin(KriaPin::Pin::EBS2, KriaPin::Direction::OUT);
    auto sdcCtrlPin= KriaPin(KriaPin::Pin::SDC_CTRL, KriaPin::Direction::OUT);
    auto asmsPin= KriaPin(KriaPin::Pin::ASMS, KriaPin::Direction::IN);
    auto sdcSensPin= KriaPin(KriaPin::Pin::SDC_SENS, KriaPin::Direction::IN);
}