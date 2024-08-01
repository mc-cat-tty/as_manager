#include <as_manager/hal/pin.hpp>
#include <iostream>
#include <stdexcept>
#include <fstream>

KriaPin::KriaPin(Pin pin, Direction dir, Value initialValue)
    : pinNumber(pin), direction(dir) {
    exportPath = "/sys/class/gpio/export";
    directionPath = "/sys/class/gpio/gpio" + std::to_string(static_cast<uint32_t>(pinNumber)) + "/direction";
    valuePath = "/sys/class/gpio/gpio" + std::to_string(static_cast<uint32_t>(pinNumber)) + "/value";

    // Export the pin
    if (!writeToFile(exportPath, std::to_string(static_cast<uint32_t>(pinNumber)))) {
        throw std::runtime_error("Failed to export GPIO pin");
    }

    // Set the direction
    setDirection(dir);

    // Set the initial value if the direction is OUT
    if (direction == Direction::OUT) {
        setValue(initialValue);
    }
}

bool KriaPin::setValue(Value value) const {
    return writeToFile(valuePath, value == Value::ON ? "1" : "0");
}

bool KriaPin::getValue() const {
    std::string valueStr = readFromFile(valuePath);
    return valueStr == "1";
}

bool KriaPin::setDirection(Direction dir) {
    direction = dir;
    return writeToFile(directionPath, dir == Direction::OUT ? "out" : "in");
}

KriaPin::Direction KriaPin::getDirection() const {
    std::string dirStr = readFromFile(directionPath);
    return dirStr == "out" ? Direction::OUT : Direction::IN;
}

bool KriaPin::writeToFile(const std::string& path, const KriaPin::Value value) const {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << path << std::endl;
        throw 1;
    }

    file << std::to_string((int)value);
    if (!file.good()) {
        std::cerr << "Failed to open file: " << path << std::endl;
        throw 1;
    }
}

bool KriaPin::readFromFile(const std::string& path) const {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << path << std::endl;
        throw  1;
    }
    int value;
    file >> value;
    return value;
}
