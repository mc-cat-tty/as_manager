#pragma once
#include <signals/interfaces.hpp>
#include <signals/low_pass_filter.hpp>
#include <hal/hal.hpp>
#include <cassert>
#include <type_traits>

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
                update();
                if constexpr (std::is_same_v<T, float>) {
                    return low_pass_filter.get_value();
                }else  if constexpr (std::is_same_v<T, bool>){
                    return hal() > 0.8f;
                }else  if constexpr (std::is_same_v<T, hal::SdcState>){
                    float value = low_pass_filter.get_value();
                    if(value> 0.8f) {
                        return hal::SdcState::Open;
                    }else if (value < 0.2f) {
                        return hal::SdcState::Closed;
                    }else
                        return hal::SdcState::Unknown;
                    
                }else
                    return hal();if constexpr (std::is_same_v<T, float>) {
                    return low_pass_filter.get_value();
                }else  if constexpr (std::is_same_v<T, bool>){
                    return hal() > 0.8f;
                }else  if constexpr (std::is_same_v<T, hal::SdcState>){
                    float value = low_pass_filter.get_value();
                    if(value> 0.8f) {
                        return hal::SdcState::Open;
                    }else if (value < 0.2f) {
                        return hal::SdcState::Closed;
                    }else if (value > 0.2f && value < 0.8f){
                        return hal::SdcState::Unknown;
                    }else
                        std::unreachable;
                }
            }

            void update() override {
                if constexpr (std::is_same_v<T, float>) {
                    low_pass_filter.update(hal());
                }else  if constexpr (std::is_same_v<T, bool>){
                    low_pass_filter.update(hal()?1.0f:0.0f);
                }else  if constexpr (std::is_same_v<T, hal::SdcState>){
                    low_pass_filter.update(hal() == hal::SdcState::Open?1.0f:0.0f);
                }
            }

        private:
            std::function<T()> hal;
            bool isFiltred;
            LowPassFilter low_pass_filter;
            IUpdater* updater;


    };
}

