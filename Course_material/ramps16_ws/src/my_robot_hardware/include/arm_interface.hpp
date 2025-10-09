#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include "SerialStreamHelper.hpp"
#include "StepperClient.hpp"
#include "StepperDriver.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

namespace arm_hardware 
    {
    class ArmHardwareInterface : public hardware_interface::SystemInterface 
    {
    public:
        // livecycle overrides
        hardware_interface::CallbackReturn
            on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn
            on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn
            on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        // system interface overrides

        hardware_interface::CallbackReturn
            on_init(const hardware_interface::HardwareInfo & info) override;
        hardware_interface::return_type
            read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
        hardware_interface::return_type
            write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        
    private:
        std::shared_ptr<SerialStreamHelper> serial_;
        int motor_id_1_;
        int motor_id_2_;
        int motor_id_3_;
        int motor_id_4_;
        int motor_id_5_;
        std::string port_;
        int baudrate_;
        float steps_per_rev_;
        bool hex_dump_;
        bool probe_;
        std::string protocol_;

        // dummy

        double motor_cal_1_;
        double motor_cal_2_;
        double motor_cal_3_;
        double motor_cal_4_;
        double motor_cal_5_;

        // Our stepper system bundle (driver, client, calibration, etc.)
        std::shared_ptr<stepper::System> sys_;

    }; // class ArmInterface     

}   // namespace arm_hardware


#endif  // arm_HARDWARE_INTERFACE_HPP



