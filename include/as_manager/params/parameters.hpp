#pragma once

namespace params {
    class Parameters {
        public:
        bool debug;
        int ebsTankPressureThreshold,
          brakePressureOneActuatorThreshold, 
          brakePressureBothActuatorsThreshold,
          brakePressureMaxonMotorsThreshold,
          unbrakePressureThreshold;
        float asmsAlpha,
          sdcAlpha,
          brakePressureFrontAlpha,
          brakePressureRearAlpha,
          rpmAlpha;

        static Parameters& getInstance() {
            static Parameters instance;
            return instance;
        }

        private:
        Parameters() = default;
        Parameters(const Parameters&) = delete;
        Parameters(Parameters&&) = delete;
        Parameters& operator=(const Parameters&) = delete;
    };
}