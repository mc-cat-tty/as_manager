#include <as_manager/hal/hal.hpp>
#include <iostream>

namespace hal{
        double random_number() {
            // Crea un generatore di numeri casuali
            std::random_device rd; // Seed random
            std::mt19937 gen(rd()); // Generatore Mersenne Twister

            std::uniform_real_distribution<> dis(0.0, 2.0);
            return dis(gen);
            
        }

        float read_ebs1_pressure(){
            if ( random_number() > 1.8)
                return 1.0f;
            else
                return 20.0f;

        }

        float read_ebs2_pressure(){
            if ( random_number() > 1.8)
                return 1.0f;
            else
                return 20.0f;

        }

        float  read_brake_pressure_front(){
            if ( random_number() > 1.2)
                return 3.0;
            else
                return 26.0;

        }


        float read_brake_pressure_rear(){
            if ( random_number() > 1.2)
                return 1.0;
            else
                return 23.0;

        }

        void toggle_watchdog_state(bool pinState){
        }

        SdcState read_sdc(){
            if (random_number() > 1.0)
                return SdcState::Open;
            else
                return SdcState::Closed;
        }

        int read_rpm(){
            if (random_number() > 0.8)
                return 4000;
            else
                return 0;
        }


        ResState read_res_state(){
            if (random_number() > 1.8)
                return ResState::Operational;
            else
                return ResState::Error;
        }

        void  toggle_actuator1_state(ActuatorState state){

        }
        void  toggle_actuator2_state(ActuatorState state){

        }

        void toggle_sdc_state(SdcState state){
        
        }

        void set_assi_Y_state(AssiState state){

        }
        void set_assi_B_state(AssiState state){
            
        }
        void set_buzzer_state(BuzzerState state){
            
        }

        void write_watchdog_state(bool pinState){
            
        }

        uint8_t read_res_bit_vector(){
            if (random_number() < 0.6)
                return 0x07;
            else if (random_number() > 0.6 && random_number() < 1.2)
                return 0x07;
            else
                return 0x07;
        }

        bool read_asms_status(){
            if (random_number() > 0.4)
                return true;
            else
                return false;
        }

        bool read_stop_message(){
            if (random_number() > 1.9)
                return true;
            else
                return false;
        }

        bool read_go_message(){
            if (random_number() > 4.0)
                return true;
            else
                return false;
        }

        bool read_mission_status(){
            if (random_number() > 1.8)
                return true;
            else
                return false;
        }

        uint8_t read_motors_bit_vector(){
            double value = random_number();
            if (value < 0.6){
                return 0x07;
            }else if (value > 0.6 && value < 1.2){
                return 0x07;
            }else{
                return 0x07;}
        }
        void send_brake_pressure_percentage(float percentage){

        }
        void send_current_state(as::EbsSupervisorState state){

        }


}