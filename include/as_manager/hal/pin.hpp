#pragma once

#include <cstdint>
#include <string>
#include <fstream>

class KriaPin {
public:
    enum class Direction {
        OUT = 0,
        IN = 1
    };

    enum class Pin: uint32_t{
        BUZZER   = 489,
        WATCHDOG = 490,
        ASSIB    = 491,
        ASSIY    = 492,
        EBS1     = 493,
        EBS2     = 494,
        SDC_CTRL = 495,
        ASMS     = 496,
        SDC_SENS = 497
    };


    enum class Value {
        ON  = 1,
        OFF = 0
    };

    KriaPin(Pin pin, Direction dir, Value initialValue = KriaPin::Value::OFF);

    bool setValue(KriaPin::Value value) const;
    KriaPin::Value getValue() const;
    bool setDirection(Direction dir);
    Direction getDirection() const;

private:
    Pin pinNumber;
    Direction direction;
    std::string exportPath;
    std::string directionPath;
    std::string valuePath;

    bool writeToFile(const std::string& path, const std::string& value) const;
    std::string readFromFile(const std::string& path) const;
};


inline KriaPin::Value operator!(KriaPin::Value v) {
  return static_cast<KriaPin::Value>(
    !static_cast<bool>(v)
  );
}