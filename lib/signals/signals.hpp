#pragma once
#include <signals/interfaces.hpp>
#include <signals/low_pass_filter.hpp>
#include <hal/hal.hpp>
#include <cassert>

namespace signals {



    using namespace signals::utils;

    template <typename T>
    class Signal : public ISignal<T> {
        public:
            Signal(std::function<T> source,  IUpdater* updater_, float alpha=1.0f) : low_pass_filter(alpha), hal(source), updater(updater_) {
                assert(alpha >= 0.0f && alpha <= 1.0f);
                if(alpha < 1.0f) {
                    updater->registerSubscriber(this);
                }
            };
            T get_value(){
                this->update();
                return low_pass_filter.get_value();
            }

            void update(){
                low_pass_filter.update(hal());
            }

        private:
            std::function<T> hal;
            LowPassFilter low_pass_filter;
            IUpdater* updater;


    };
}

