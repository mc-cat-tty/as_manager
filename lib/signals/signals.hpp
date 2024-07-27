#pragma once
#include <signals/interfaces.hpp>
#include <signals/low_pass_filter.hpp>
#include <cassert>
#include <type_traits>
#include <utility>

namespace signals {
  using namespace signals::utils;

  /**
   * @brief Proxy object to low pass filter and its updater.
   * @param alpha Set alpha to 1.0 in order to disable filtering.
   */
  template <typename T>
  class Signal : public IUpdatable {
      public:
          constexpr Signal(std::invocable auto signal_source, IUpdater *updater, float alpha = 1.0f) :
            low_pass_filter(alpha), signal_source(signal_source), updater(updater) {
              if (alpha == 1.0f) return;
              updater->registerSubscriber(this);
          };

          float get_value() {
            if (low_pass_filter.get_alpha() == 1.0f)
              return static_cast<float>(signal_source());
            else
              return low_pass_filter.get_value();
          }

          void update() {
            if (low_pass_filter.get_alpha() < 1.0f)
              low_pass_filter.update(signal_source());
          }

      private:
          std::function<T()> signal_source;
          LowPassFilter<T> low_pass_filter;
          IUpdater* updater;
  };
}
