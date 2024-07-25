#ifndef LIB_HAL_HAL_HPP
#define LIB_HAL_HAL_HPP

#include <cstdint>
#include <common/common_type.hpp>
#include <random>
#include <chrono>

namespace as{
  namespace hal{
    class Hal  {
    public:
        Hal();


        SdcState read_sdc();
        bool read_asms_status() ;

        void toggle_actuator1_state(ActuatorState state) ;
        void toggle_actuator2_state(ActuatorState state) ;
        void toggle_watchdog_state();
        void set_assi_Y_state(AssiState state);
        void set_assi_B_state(AssiState state);
        void set_buzzer_state(BuzzerState state);
        void toggle_sdc_state(SdcState state);

        uint8_t read_res_bit_vector();
        ResState read_res_state();
        float read_brake_pressure_front() ;
        float read_brake_pressure_rear() ;
        int read_rpm();
        float read_ebs1_pressure() ;
        float read_ebs2_pressure() ;
        bool read_stop_message() ;
        uint8_t read_motors_bit_vector();

        void send_brake_pressure_percentage(float percentage);
        void send_current_state(EbsSupervisorState state);



    private:
        const unsigned int actuator1 = 0;
        const unsigned int actuator2 = 1;
        const unsigned int brake_pressure = 2;
        const unsigned int watchdog = 3;
        const unsigned int sdc = 4;

        //variabili di test
        int count_read_ebs1_pressure;
        int count_read_ebs2_pressure;
        int count_read_res;
        int count_read_sdc;
        int count_read_pressure_breake_front;
        int count_read_pressure_breake_gear;

        double random_number();
        
    };

    typedef enum MmrPinState {
      MMR_PIN_LOW,
      MMR_PIN_HIGH,
    } MmrPinState;
  };
};


#endif //MMR_HAL_H