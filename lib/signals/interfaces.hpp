namespace signals {
  struct IUpdatable {
    virtual void update() = 0;
  };

  struct IUpdater{
    virtual void registerSubscriber(IUpdatable* subscriber) = 0;
    virtual void update() = 0;
  };
}