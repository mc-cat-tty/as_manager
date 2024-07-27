#pragma once
#include <signals/interfaces.hpp>
#include <signals/low_pass_filter.hpp>
#include <signal_source/signal_source.hpp>
#include <cassert>
#include <type_traits>
#include <utility>

namespace signals {
    using namespace signals::utils;

    template <typename T>
    class Signal : public IUpdatable {
        public:
            Signal(std::invocable<T> auto signal_source,  IUpdater* updater_, float alpha=1.0f) : low_pass_filter(alpha), signal_source(source), updater(updater_) {
                if (alpha == 1.0f) return;
                updater->registerSubscriber(this);
            };

            T get_value() {
              if (low_pass_filter->get_alpha() == 1.0f)
                return signal_source();
              else
                return low_pass_filter.get_value();
            }

            void update() override {
              if (low_pass_filter->get_alpha() < 1.0f)
                low_pass_filter.update(signal_source());
            }

        private:
            std::function<T()> signal_source;
            LowPassFilter low_pass_filter;
            IUpdater* updater;


    };
}

Signal(thresholdDecorate(sdc_hal, 0.6), updater, 0.2);