#pragma once
#include <cstdint>

namespace hal::utils {

    enum class Motors{
        Clutch=0x01,
        Stear=0x02,
        Brake=0x04,
        All=0x07
    };

    bool motorMask(Motors motor, uint8_t bitVector){
        return (bitVector & static_cast<uint8_t>(motor));
    }
}
