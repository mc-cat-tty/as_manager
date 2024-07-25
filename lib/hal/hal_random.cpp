#include <hal/hal.hpp>

namespace as{
    namespace hal{
        double Hal::random_number() {
            // Crea un generatore di numeri casuali
            std::random_device rd; // Seed random
            std::mt19937 gen(rd()); // Generatore Mersenne Twister

            std::uniform_real_distribution<> dis(0.0, 2.0);
            return dis(gen);
            
        }

        Hal::Hal(){
            /*
                TODO: inizializza i PIN
            */
        count_read_ebs1_pressure = 0;
        count_read_ebs2_pressure = 0;
        count_read_res=0;
        count_read_sdc=0;
        count_read_pressure_breake_front=0;
        count_read_pressure_breake_gear=0;
        }

        float Hal::read_ebs1_pressure(){
            if ( random_number() > 1.0)
                return 1.0f;
            else
                return 3.0f;

        }

        float Hal::read_ebs2_pressure(){
            if ( random_number() > 1.0)
                return 1.0f;
            else
                return 3.0f;

        }

        float  Hal::read_brake_pressure_front(){
            if ( random_number() > 1.8)
                return 3.0;
            else
                return 13.0;

        }


        float Hal::read_brake_pressure_rear(){
            if ( random_number() > 1.8)
                return 1.0;
            else
                return 12.0;

        }

        void Hal::toggle_watchdog_state(){
        }

        SdcState Hal::read_sdc(){
            if (random_number() > 0.2)
                return SdcState::Open;
            else
                return SdcState::Closed;
        }

        int Hal::read_rpm(){
            if (random_number() > 0.8)
                return 4000;
            else
                return 0;
        }


        ResState Hal::read_res_state(){
            if (random_number() > 1.8)
                return ResState::Operational;
            else
                return ResState::Error;
        }

        void  Hal::toggle_actuator1_state(ActuatorState state){

        }
        void  Hal::toggle_actuator2_state(ActuatorState state){

        }

        void Hal::toggle_sdc_state(SdcState state){
        
        }

        void Hal::set_assi_Y_state(AssiState state){

        }
        void Hal::set_assi_B_state(AssiState state){
            
        }
        void Hal::set_buzzer_state(BuzzerState state){
            
        }

        uint8_t Hal::read_res_bit_vector(){
            if (random_number() < 0.6)
                return 0x00;
            else if (random_number() > 0.6 && random_number() < 1.2)
                return 0x01;
            else
                return 0x03;
        }

        bool Hal::read_asms_status(){
            if (random_number() > 0.4)
                return true;
            else
                return false;
        }

        bool Hal::read_stop_message(){
            if (random_number() > 1.8)
                return true;
            else
                return false;
        }

        uint8_t Hal::read_motors_bit_vector(){
            if (random_number() < 0.6)
                return 0x00;
            else if (random_number() > 0.6 && random_number() < 1.2)
                return 0x01;
            else
                return 0x03;
        }
        void Hal::send_brake_pressure_percentage(float percentage){

        }
        void Hal::send_current_state(EbsSupervisorState state){

        }


    }
}