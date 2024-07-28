#pragma once
#include <cstdint>

namespace hal::utils {

    enum class Motors{
        Clutch=0x01,
        Stear=0x02,
        Brake=0x04,
        All=0x07
    };

    enum class ResBitVector{
        Go=0x01,
        Bag=0x02,
        Emergency=0x04
    };

    bool motorMask(Motors motor, uint8_t data){
        auto motorBitvector = static_cast<uint8_t>(motor);
        return (data & motorBitvector) == motorBitvector;
    }

    bool resmask(ResBitVector resState, uint8_t data){
        auto resBitVector= static_cast<uint8_t>(resState);
        return (data & resBitVector) == resBitVector;
    }
}
