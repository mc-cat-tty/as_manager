#include <as_manager/watchdog/watchdog.hpp>
#include <as_manager/assi_manager/assi_manager.hpp>
#include <as_manager/ebs_supervisor/ebs_supervisor.hpp>
#include <as_manager/signals/updater.hpp>


int main() {
    auto& watchdog= watchdog::Watchdog::getInstance();
    auto& assi_manager= as::assi_manager::AssiManager::getInstance();
    auto EbsSupervisor= as::ebs_supervisor::EbsSupervisor();
    auto& updater= signals::utils::Updater<5>::getInstance();

    while(1){
        updater.update();
        EbsSupervisor.run();
        watchdog.run();
        assi_manager.run();
    }
}