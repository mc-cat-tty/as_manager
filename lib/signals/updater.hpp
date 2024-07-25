#pragma once

#include <signals/signals.hpp>

namespace signals::utils{

    class IUpdater{
        public:
            virtual void registerSubscriber(void* subscriber) = 0;

    };

    template <int maxSubscribers>
    class Updater : public IUpdater {
        public:
            static Updater& getInstance() {
                static Updater instance;
                return instance;
            }

            void registerSubscriber( signals::ISignal<>* updateMethod) {
                if (numSubscribers < maxSubscribers) {
                    updateMethods[numSubscribers++] = updateMethod;
                }
            }

            void update() {
                for (int i = 0; i < numSubscribers; ++i) {
                    updateMethods[i]->get_value();
                }
            }

        private:
            Updater() = default;
            ~Updater() = default;
            Updater(const Updater&) = delete;
            Updater& operator=(const Updater&) = delete;

            ISignal* updateMethods[maxSubscribers];
            int numSubscribers = 0;
    
    };


}