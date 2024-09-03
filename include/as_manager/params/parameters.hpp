#pragma once
#include <rclcpp/rclcpp.hpp>
#include <string_view>
#include <chrono>

namespace params {
  class Parameters {
    // Parameters() = default;
    // Parameters(const Parameters&) = delete;
    // Parameters(Parameters&&) = delete;
    // Parameters& operator=(const Parameters&) = delete;

    public:
    bool safetyFeatures, verboseCallbacks, verboseHalReads;

    int ebsTankPressureThreshold,
      brakePressureOneActuatorThreshold, 
      brakePressureBothActuatorsThreshold,
      brakePressureMaxonMotorsThreshold,
      unbrakePressureThreshold,
      rpmTsActiveThreshold;
    
    std::chrono::milliseconds assertEbsPressureTimeoutMs,
      assertAllBrakesPressureTimeoutMs,
      assertOneBrakePressureTimeoutMs,
      assertNoBrakePressureTimeoutMs,
      assertMaxonBrakePressureTimeoutMs,
      assertSdcOpenTimeoutMs,
      assertSdcCloseTimeoutMs,
      continousMonitoringTimeoutsMs;
    
    float brakePressureFrontAlpha,
      brakePressureRearAlpha,
      rpmAlpha;
    
    float maxonBrakePressure;

    static Parameters& getInstance() {
      static Parameters instance;
      return instance;
    }
  };

  class ParametersProxy {
    rclcpp::Node &node;
    std::string_view prefix;

    inline std::string getName(std::string_view param_name) const {
      return prefix == "" ? std::string(param_name) : std::string(prefix) + "." + std::string(param_name);
    }

    public:
    ParametersProxy(std::string_view prefix, rclcpp::Node* node) : node(*node), prefix(prefix) {};

    template<typename T>
    T get(std::string_view paramName, std::optional<T> defVal = std::nullopt) const {
      auto _paramName = getName(paramName);

      try {
        if (not node.has_parameter(_paramName)) node.declare_parameter(_paramName, defVal.value_or(T{}));
        return node.get_parameter(_paramName).get_value<T>();
      }
      catch (...) {
        RCLCPP_ERROR(node.get_logger(), "Erorr while parsing parameter '%s'", _paramName.c_str());
        throw;      
      }
    }

    template<typename T, typename CastType>
    CastType get(std::string_view paramName, std::optional<T> defVal = std::nullopt) const {
      return CastType{get(paramName, defVal)};
    }

    template<typename T>
    T get(std::string_view paramName, T val) const {
      return get(paramName, std::optional{val});
    }
  };
}