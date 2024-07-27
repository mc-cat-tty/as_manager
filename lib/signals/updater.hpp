#pragma once

#include <signals/signals.hpp>
#include <iostream>

namespace signals::utils{
  template <int maxSubscribers>
  class Updater : public IUpdater {
      public:
          static Updater& getInstance() {
              static Updater instance;
              return instance;
          }

          void registerSubscriber(IUpdatable* subscriber) override {
              if (numSubscribers < maxSubscribers) {
                  subscribers[numSubscribers++] = subscriber;
              }
          }

          void update() override {
              //std::cout<<"[UPDATER] update"<<std::endl;
              for (int i = 0; i < numSubscribers; ++i) {
                  subscribers[i]->update();
              }
          }

      private:
          Updater() = default;
          ~Updater() = default;
          Updater(const Updater&) = delete;
          Updater& operator=(const Updater&) = delete;

          IUpdatable *subscribers[maxSubscribers];
          int numSubscribers = 0;  
  };
}