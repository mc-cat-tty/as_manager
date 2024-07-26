#pragma once

#include <signals/signals.hpp>

namespace signals::utils{
  template <int maxSubscribers>
  class Updater : public IUpdater {
      public:
          static Updater& getInstance() {
              static Updater instance;
              return instance;
          }

          void registerSubscriber(void* subscriber) override {
              if (numSubscribers < maxSubscribers) {
                  subscribers[numSubscribers++] = subscriber;
              }
          }

          void update() override {
              for (int i = 0; i < numSubscribers; ++i) {
                  subscribers[i]->get_value();
              }
          }

      private:
          Updater() = default;
          ~Updater() = default;
          Updater(const Updater&) = delete;
          Updater& operator=(const Updater&) = delete;

          void *subscribers[maxSubscribers];
          int numSubscribers = 0;  
  };
}