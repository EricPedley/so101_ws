#ifndef CREATE2_HARDWARE_INTERFACE_HPP
#define CREATE2_HARDWARE_INTERFACE_HPP

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "feetech_lib.hpp"

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace so101_hardware
{
    class SO101HardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(SO101HardwareInterface);

        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // Serial communication
        int serial_fd_;
        std::string device_path_;
        std::string calibration_path;
        int baud_rate_;
        FeetechServo servo;
        std::vector<int> servo_ids;
        int homing_offsets[6];
        double servo_position_states[6];
        double servo_position_commands[6];
        std::vector<std::string> joint_names {"shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"};
    };

} // namespace create2_hardware

#endif