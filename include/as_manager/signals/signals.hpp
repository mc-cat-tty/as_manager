#pragma once
#include <as_manager/signals/interfaces.hpp>
#include <as_manager/signals/low_pass_filter.hpp>
#include <cassert>
#include <type_traits>
#include <utility>

namespace signals {
  using namespace signals::utils;

    enum class ValueRespectTreshold {
    LOWER = 0,
    BIGGER
  };

  /**
   * @brief Proxy object to low pass filter and its updater.
   * @param alpha Set alpha to 1.0 in order to disable filtering.
   */
  template <typename T>
  class Signal : public IUpdatable {
      public:
          constexpr Signal(std::invocable auto signal_source, IUpdater *updater, float &alpha) :
            low_pass_filter(alpha), signal_source(signal_source), updater(updater) {
              if (alpha == 1.0f) return;
              updater->registerSubscriber(this);
          };

          template <ValueRespectTreshold vrt=ValueRespectTreshold::LOWER>
          bool get_value_with_threhold(float threshold) {
                if constexpr (vrt == ValueRespectTreshold::LOWER)
                    
                    return low_pass_filter.get_value() < threshold;
                else
                    return low_pass_filter.get_value() > threshold;
                
          }

          float get_value() {
            if (low_pass_filter.get_alpha() == 1.0f)
              return static_cast<float>(signal_source());
            else
              return low_pass_filter.get_value();
          }

          void update() {
            std::cout << "ALPHA: " << low_pass_filter.get_alpha() << std::endl;
            if (low_pass_filter.get_alpha() < 1.0f)
              low_pass_filter.update(signal_source());
          }

      private:
          std::function<T()> signal_source;
          LowPassFilter<T> low_pass_filter;
          IUpdater* updater;
  };
}
