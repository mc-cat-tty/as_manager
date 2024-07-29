#include <rclcpp/rclcpp.hpp>

#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/signals/updater.hpp>

constexpr unsigned updatableSignalsNumber = 5;


class AsManagerNode : public rclcpp::Node {
    private:
    as::ebs_supervisor::EbsSupervisor ebsSupervisor;
    watchdog::Watchdog &watchdog;
    as::assi_manager::AssiManager &assiManager;
    signals::utils::Updater<updatableSignalsNumber> &signalUpdater;

    void superLoop();

    public:
    AsManagerNode();
    ~AsManagerNode();
};