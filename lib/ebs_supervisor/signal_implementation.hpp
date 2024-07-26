#include <signals/signals.hpp>
#include <signals/updater.hpp>
#include <hal/hal.hpp>

namespace as::ebs_supervisor{
    using namespace signals;
    inline auto& updater = utils::Updater<20>::getInstance();

    auto asms_signal= Signal<bool>(hal::read_asms_status,&updater);
    auto mission_signal= Signal<bool>(hal::read_mission_status,&updater);
    auto ebs1_signal = Signal<float>(hal::read_ebs1_pressure,&updater);
    auto ebs2_signal = Signal<float>(hal::read_ebs2_pressure,&updater);
    auto sdc_signal = Signal<hal::SdcState>(hal::read_sdc,&updater);
    auto breake_pressure_front_signal = Signal<float>(hal::read_brake_pressure_front,&updater);
    auto breake_pressure_rear_signal = Signal<float>(hal::read_brake_pressure_rear,&updater);
    auto motors_bit_vector_singal = Signal<uint8_t>(hal::read_motors_bit_vector,&updater);
    
}