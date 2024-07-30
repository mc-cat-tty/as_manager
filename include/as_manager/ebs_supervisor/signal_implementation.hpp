#pragma once

#include <as_manager/signals/signals.hpp>
#include <as_manager/signals/updater.hpp>
#include <as_manager/hal/hal.hpp>
#include <as_manager/as_manager.hpp>

namespace as::ebs_supervisor{
    using namespace signals;
    inline auto& updater = signals::utils::Updater<5>::getInstance();

    // Over the threshold
    constexpr uint8_t EXPECTED_PRESSURE_EBS_TANK = 5;
    constexpr uint8_t EXPECTED_BRAKE_PRESSURE_ONE_ACTUATOR = 10;
    constexpr uint8_t EXPECTED_BRAKE_PRESSURE_BOTH_ACTUATORS = 20;
    constexpr uint8_t EXPECTED_BRAKE_PRESSURE_MAXON_MOTOR = 6;


    // Under the threshold
    constexpr uint8_t EXPECTED_UNBRAKE_PRESSURE = 5;

    const inline auto ASMS_THRESHOLD = 0.9f;
    const inline auto SDC_TRESHOLD_OPEN = 0.9f;
    const inline auto SDC_TRESHOLD_CLOSE = 0.1f;

    auto asms_signal= Signal<bool>(hal::read_asms_status,&updater,AsManagerNode::getAsmsAlpha());
    auto mission_signal= Signal<bool>(hal::is_autonomous_mission,&updater);
    auto ebs1_signal = Signal<float>(hal::read_ebs1_pressure,&updater);
    auto ebs2_signal = Signal<float>(hal::read_ebs2_pressure,&updater);
    auto sdc_signal = Signal<hal::SdcState>(hal::read_sdc,&updater,AsManagerNode::getSdcAlpha());
    auto breake_pressure_front_signal = Signal<float>(hal::read_brake_pressure_front,&updater,AsManagerNode::getBrakePressureFrontAlpha());
    auto breake_pressure_rear_signal = Signal<float>(hal::read_brake_pressure_rear,&updater,AsManagerNode::getBrakePressureRearAlpha());
    auto motors_bit_vector_singal = Signal<uint8_t>(hal::read_motors_bit_vector,&updater);
    auto go_signal= Signal<bool>(hal::read_go_message,&updater);
    auto stop_signal= Signal<bool>(hal::read_stop_message,&updater);
    auto res_emergency_signal= Signal<hal::ResState>(hal::read_res_state,&updater);
    auto res_bit_vector_signal = Signal<uint8_t>(hal::read_res_bit_vector,&updater);
    auto rpm_signal= Signal<float>(hal::read_rpm,&updater,AsManagerNode::getRpmAlpha());
    


}