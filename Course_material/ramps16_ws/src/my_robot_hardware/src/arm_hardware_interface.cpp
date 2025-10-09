#include "arm_interface.hpp"

namespace arm_hardware
    {

    // -----------------------------------------------------------------------------
    // on_init(): called once when the plugin is created by ros2_control.
    // - Read and cache metadata from URDF / hardware_info (via base class).
    // - Parse parameters you care about (port, baud, IDs, etc.).
    // - DO NOT touch real hardware here — just prepare configuration.
    // - Build the shared 'System' bundle (but do not open the serial port yet).
    // -----------------------------------------------------------------------------

    hardware_interface::CallbackReturn
        ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)

        {
            if (hardware_interface::SystemInterface::on_init(info)
                != hardware_interface::CallbackReturn::SUCCESS) {
                return hardware_interface::CallbackReturn::ERROR;
            }

            // Pull params from <hardware> tag in your ros2_control yaml/URDF
            auto get_s = [&](const char* k, const std::string& d="") {
                auto it = info_.hardware_parameters.find(k);
                return it != info_.hardware_parameters.end() ? it->second : d;
            };
            auto get_u = [&](const char* k, unsigned d) {
                auto s = get_s(k);
                try { return s.empty() ? d : static_cast<unsigned>(std::stoul(s)); }
                catch (...) { return d; }
            };
            auto get_d = [&](const char* k, double dflt) {
                auto s = get_s(k);
                try { return s.empty() ? dflt : std::stod(s); }
                catch (...) { return dflt; }
            };
            auto get_b = [&](const char* k, bool dflt) {
                auto s = get_s(k);
                if (s == "1" || s == "true" || s == "True") return true;
                if (s == "0" || s == "false"|| s == "False") return false;
                return dflt;
            };

            const std::string port      = get_s("port", "/dev/ttyACM0");
            const unsigned    baud      = get_u("baudrate", 57600);
            const uint8_t     id1       = static_cast<uint8_t>(get_u("motor_id_1", 1));
            const uint8_t     id2       = static_cast<uint8_t>(get_u("motor_id_2", 2));
            const uint8_t     id3       = static_cast<uint8_t>(get_u("motor_id_3", 3));
            const uint8_t     id4       = static_cast<uint8_t>(get_u("motor_id_4", 4));
            const uint8_t     id5       = static_cast<uint8_t>(get_u("motor_id_5", 5));
            const double      spr       = get_d("steps_per_rev", 200.0);
            const bool        hex_dump  = get_b("hex_dump", false);
            const bool        probe     = get_b("probe", false);
            const std::string proto     = get_s("protocol", "register");

            // Build the config-only bundle
            sys_ = std::make_shared<stepper::System>(
                    stepper::StepperDriver::Prepare(
                        port, baud, id1, id2, id3, id4, id5, spr, hex_dump, probe, proto));

            RCLCPP_INFO(get_logger(),
                "Prepared Stepper System: device='%s' baud=%u id1=%u id2=%u id3=%u id4=%u id5=%u spr=%.1f hex=%d probe=%d proto=%s",
                sys_->device.c_str(), sys_->baud,
                sys_->motor_id_1, sys_->motor_id_2,
                sys_->motor_id_3, sys_->motor_id_4,
                sys_->motor_id_5, sys_->steps_per_rev,
                sys_->hex_dump, sys_->probe, sys_->protocol.c_str());

            return hardware_interface::CallbackReturn::SUCCESS;
    }
    
// -----------------------------------------------------------------------------
// on_configure(): called when the ResourceManager transitions hardware from
// UNCONFIGURED -> INACTIVE.
// - This is where we touch the real hardware for the first time.
// - Open the serial port and initialize the client/driver.
// -----------------------------------------------------------------------------

hardware_interface::CallbackReturn
ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State&)
{
  if (!sys_) {
    RCLCPP_ERROR(get_logger(), "System bundle (sys_) is null in on_configure()");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (sys_->device.empty()) {
    RCLCPP_ERROR(get_logger(), "Serial device parameter is empty");
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    stepper::StepperDriver::Init(*sys_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "StepperDriver::Init failed: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Stepper system initialized.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
    ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State&)
    {
    if (!sys_ || !sys_->ready() || !sys_->driver) {
        RCLCPP_ERROR(get_logger(), "Driver not ready in on_activate()");
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto& drv = *sys_->driver;
    if (drv.activateWithVelocityMode(sys_->motor_id_1) != 0) return hardware_interface::CallbackReturn::ERROR;
    if (drv.activateWithVelocityMode(sys_->motor_id_2) != 0) return hardware_interface::CallbackReturn::ERROR;
    if (drv.activateWithVelocityMode(sys_->motor_id_3) != 0) return hardware_interface::CallbackReturn::ERROR;
    if (drv.activateWithVelocityMode(sys_->motor_id_4) != 0) return hardware_interface::CallbackReturn::ERROR;
    if (drv.activateWithVelocityMode(sys_->motor_id_5) != 0) return hardware_interface::CallbackReturn::ERROR;
 
    // At this stage the hardware is connected an we need to set
    // valid initial values for the state variables to prevent
    // NaN values in the state broadcaster.

    set_state("axis_1/velocity",0.0); 
    set_state("axis_2/velocity",0.0); 
    set_state("axis_3/velocity",0.0); 
    set_state("axis_4/velocity",0.0); 
    set_state("axis_5/velocity",0.0); 
    set_state("axis_1/position",0.0); 
    set_state("axis_2/position",0.0); 
    set_state("axis_3/position",0.0); 
    set_state("axis_4/position",0.0); 
    set_state("axis_5/position",0.0); 
 
    return hardware_interface::CallbackReturn::SUCCESS;
    }

  
    hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate
        (const rclcpp_lifecycle::State & previous_state) 
        {
            (void)previous_state;

            // Stop

            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_1, 0.0, sys_->cal) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_2, 0.0, sys_->cal) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_3, 0.0, sys_->cal) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_4, 0.0, sys_->cal) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_5, 0.0, sys_->cal) != 0)
                return hardware_interface::CallbackReturn::ERROR;
 
 
            // wait a bit
            std::this_thread::sleep_for(std::chrono::milliseconds(300));

            // Disable

            if (sys_->driver->deactivate(motor_id_1_) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            if (sys_->driver->deactivate(motor_id_2_) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            if (sys_->driver->deactivate(motor_id_3_) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            if (sys_->driver->deactivate(motor_id_4_) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            if (sys_->driver->deactivate(motor_id_5_) != 0)
                return hardware_interface::CallbackReturn::ERROR;

            return hardware_interface::CallbackReturn::SUCCESS;

        }

    hardware_interface::return_type ArmHardwareInterface::read
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
        {
            (void)time;

            auto& drv = *sys_->driver;
            const auto& cal = sys_->cal;

            // read the current velocity of both motors
            // and update the state variables accordingly
            // we assume that the velocity is constant over the period
            // so we can integrate the position by pos = pos + vel * dt

            double vel_motor_1_ = drv.getVelocityRadianPerSec(sys_->motor_id_1, cal);
            //int32_t p1 = drv.getPositionSteps(sys_->motor_id_1);
            double vel_motor_2_ = drv.getVelocityRadianPerSec(sys_->motor_id_2, cal);
            //int32_t p2 = drv.getPositionSteps(sys_->motor_id_2);
            double vel_motor_3_ = drv.getVelocityRadianPerSec(sys_->motor_id_3, cal);
            //int32_t p1 = drv.getPositionSteps(sys_->motor_id_1);
            double vel_motor_4_ = drv.getVelocityRadianPerSec(sys_->motor_id_4, cal);
            //int32_t p2 = drv.getPositionSteps(sys_->motor_id_2);
            double vel_motor_5_ = drv.getVelocityRadianPerSec(sys_->motor_id_5, cal);
            //int32_t p1 = drv.getPositionSteps(sys_->motor_id_1);
 

            if (abs(vel_motor_1_) < 0.03) { vel_motor_1_ = 0.0; }
            if (abs(vel_motor_2_) < 0.03) { vel_motor_2_ = 0.0; }
            if (abs(vel_motor_3_) < 0.03) { vel_motor_3_ = 0.0; }
            if (abs(vel_motor_4_) < 0.03) { vel_motor_4_ = 0.0; }
            if (abs(vel_motor_5_) < 0.03) { vel_motor_5_ = 0.0; }

            auto steps1_ = sys_->driver->getPositionSteps(sys_->motor_id_1);
            auto rad1_   = sys_->driver->getPositionRadian(sys_->motor_id_1, sys_->cal);
            auto steps2_ = sys_->driver->getPositionSteps(sys_->motor_id_2);
            auto rad2_   = sys_->driver->getPositionRadian(sys_->motor_id_2, sys_->cal);
            auto steps3_ = sys_->driver->getPositionSteps(sys_->motor_id_3);
            auto rad3_   = sys_->driver->getPositionRadian(sys_->motor_id_3, sys_->cal);
            auto steps4_ = sys_->driver->getPositionSteps(sys_->motor_id_4);
            auto rad4_   = sys_->driver->getPositionRadian(sys_->motor_id_4, sys_->cal);
            auto steps5_ = sys_->driver->getPositionSteps(sys_->motor_id_5);
            auto rad5_   = sys_->driver->getPositionRadian(sys_->motor_id_5, sys_->cal);

            //RCLCPP_INFO(get_logger(), "steps1=%d rad1=%.3f steps2=%d rad2=%.3f", steps1_, rad1_, steps2_, rad2_);

            //double vel_motor_1_ = sys_->driver->getVelocityRadianPerSec(sys_->motor_id_1, sys_->cal); 
            //double vel_motor_2_ = sys_->driver->getVelocityRadianPerSec(sys_->motor_id_2, sys_->cal); 
            //double vel_motor_1_ = sys_->driver->getVelocityRadianPerSec(motor_id_1_, motor_cal_1_); 
            //double vel_motor_2_ = sys_->driver->getVelocityRadianPerSec(motor_id_2_, motor_cal_2_); 

            set_state("axis_1/velocity",vel_motor_1_); // from xacro
            set_state("axis_2/velocity",vel_motor_2_); // from xacro
            set_state("axis_3/velocity",vel_motor_3_); // from xacro
            set_state("axis_4/velocity",vel_motor_4_); // from xacro
            set_state("axis_5/velocity",vel_motor_5_); // from xacro

            set_state("axis_1/position",  get_state("axis_1/position") + vel_motor_1_* period.seconds());
            set_state("axis_2/position",  get_state("axis_2/position") + vel_motor_2_* period.seconds());
            set_state("axis_3/position",  get_state("axis_3/position") + vel_motor_3_* period.seconds());
            set_state("axis_4/position",  get_state("axis_4/position") + vel_motor_4_* period.seconds());
            set_state("axis_5/position",  get_state("axis_5/position") + vel_motor_5_* period.seconds());
 
            RCLCPP_INFO(get_logger(), "pos1=%.3f pos2=%.3f pos3=%.3f pos4=%.3f pos5=%.3f vel1=%.3f vel2=%.3f vel3=%.3f vel4=%.3f vel5=%.3f",
             rad1_, rad2_, rad3_, rad4_, rad5_,
             vel_motor_1_, vel_motor_2_,vel_motor_3_, vel_motor_4_,vel_motor_5_);

            return hardware_interface::return_type::OK;
        }
    hardware_interface::return_type ArmHardwareInterface::write
        (const rclcpp::Time & time, const rclcpp::Duration & period) 
        {
            (void)time;
            (void)period;

            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_1, get_command("axis_1/velocity"), sys_->cal) != 0)
               return hardware_interface::return_type::ERROR;
            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_2, get_command("axis_2/velocity"), sys_->cal) != 0)
               return hardware_interface::return_type::ERROR;
            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_3, get_command("axis_3/velocity"), sys_->cal) != 0)
               return hardware_interface::return_type::ERROR;
            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_4, get_command("axis_4/velocity"), sys_->cal) != 0)
               return hardware_interface::return_type::ERROR;
            if (sys_->driver->setTargetVelocityRadianPerSec(sys_->motor_id_5, get_command("axis_5/velocity"), sys_->cal) != 0)
               return hardware_interface::return_type::ERROR;

            RCLCPP_INFO(get_logger(), "axis 1 vel: %lf, axis 2 vel: %lf, axis 3 vel: %lf, axis 4 vel: %lf axis 5 vel: %lf", 
            get_command("axis_1/velocity"), get_command("axis_2/velocity"), get_command("axis_3/velocity"),
            get_command("axis_4/velocity"), get_command("axis_5/velocity"));
        return hardware_interface::return_type::OK;

        }
} // namespace arm_hardware


// The hardware interface written in c above will be loaded by the controller manager
// at runtime because we will build is as a plug-in. The controller manager will load
// all controllers and the hardware interface at runtime.
// We need to set the ArmHardwareInterface class as a plugin to be loaded.
// The header files need not be changed but the cpp file needs to have the following
// lines at the end of the file.

// Start with an include 

#include "pluginlib/class_list_macros.hpp"

// Macro code, include namespace and class name and then the second argument is the parent class we inherit from.

PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)  
