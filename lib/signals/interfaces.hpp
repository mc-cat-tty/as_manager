namespace signals {
  template <typename T>
  struct ISignal {
    virtual T get_value() = 0;
    virtual void update();
  };

  struct IUpdater{
    virtual void registerSubscriber(void* subscriber) = 0;
    virtual void update() = 0;
  };
}