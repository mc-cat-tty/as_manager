#pragma once
#include <signals/interfaces.hpp>
#include <signals/low_pass_filter.hpp>
#include <hal/hal.hpp>
#include <cassert>

namespace signals {
    using namespace signals::utils;

    template <typename T>
    class Signal : public IUpdatable {
        public:
            Signal(std::function<T()> source,  IUpdater* updater_, float alpha=1.0f) : isFiltred(false),low_pass_filter(alpha), hal(source), updater(updater_) {
                assert(alpha >= 0.0f && alpha <= 1.0f);
                if(alpha < 1.0f) {
                    updater->registerSubscriber(this);
                    isFiltred = true;  
                }
            };
            T get_value() {
                if( isFiltred )
                    return low_pass_filter.get_value();
                else
                    return hal();
            }

            void update() override {
                low_pass_filter.update(static_cast<float>(hal()));
            }

        private:
            std::function<T()> hal;
            bool isFiltred;
            LowPassFilter low_pass_filter;
            IUpdater* updater;


    };
}

