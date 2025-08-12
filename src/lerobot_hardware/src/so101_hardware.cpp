#include "lerobot_hardware/so101_hardware.hpp"

#include <limits>
#include <vector>
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp"

namespace so101_hardware
{
    hardware_interface::CallbackReturn SO101HardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Get device path from hardware parameters
        device_path_ = info_.hardware_parameters["device"];
        calibration_path = info_.hardware_parameters["calibration_file"];
        baud_rate_ = std::stoi(info_.hardware_parameters.count("baud_rate") ? info_.hardware_parameters["baud_rate"] : "115200");

        // TODO: read servo_ids and homing offsets from json file with this format:
        // {
        //     "shoulder_pan": {
        //         "id": 1,
        //         "drive_mode": 0,
        //         "homing_offset": -1768,
        //         "range_min": 625,
        //         "range_max": 3337
        //     },

        

        // Check that we have the expected joints
        if (info_.joints.size() != 6)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("SO101HardwareInterface"),
                "Expected exactly 6 joints, got %zu", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("SO101HardwareInterface"),
            "Initialized Create2 hardware interface with device: %s", device_path_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SO101HardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // example:
        // state_interfaces.emplace_back(hardware_interface::StateInterface(
        //     info_.joints[0].name, "position", &left_wheel_position_));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SO101HardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        //example:
        // command_interfaces.emplace_back(hardware_interface::CommandInterface(
        //     info_.joints[0].name, "velocity", &left_wheel_velocity_command_));


        return command_interfaces;
    }

    hardware_interface::CallbackReturn SO101HardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("SO101HardwareInterface"), "Activating...");

        FeetechServo servo(device_path_, baud_rate_, 30, servo_ids, false);

        // servo.setOperatingMode(servo_ids[0], DriverMode::CONTINUOUS_POSITION);


        // TODO: for each servo do this
        // servo.setOperatingMode(servo_ids[0], DriverMode::CONTINUOUS_POSITION);

        RCLCPP_INFO(rclcpp::get_logger("SO101HardwareInterface"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SO101HardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("SO101HardwareInterface"), "Deactivating...");

        servo.close();

        RCLCPP_INFO(rclcpp::get_logger("SO101HardwareInterface"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type SO101HardwareInterface::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // TODO: read servo state into memory

        // ex:
        // std::vector<double> current_position = servo.getCurrentPositions();
        // std::cout<< "Got position: " << current_position[0] << std::endl;
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SO101HardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        // TODO: write positions to servos

        // ex:
        // servo.setReferencePosition(servo_ids[0], current_position[0]);
        return hardware_interface::return_type::OK;
    }


} // namespace so101_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    create2_hardware::SO101HardwareInterface, hardware_interface::SystemInterface)