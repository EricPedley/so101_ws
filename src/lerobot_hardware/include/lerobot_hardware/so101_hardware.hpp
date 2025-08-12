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

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

namespace create2_hardware
{
class Create2HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Create2HardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial communication
  int serial_fd_;
  std::string device_path_;
  int baud_rate_;
  
  // Robot parameters
  const double wheel_radius_ = 0.036;  // meters
  const double wheel_separation_ = 0.235;  // meters
  const double counts_per_rev_ = 508.8;  // encoder counts per wheel revolution
  
  // State variables
  double left_wheel_position_;
  double right_wheel_position_;
  
  // Command variables
  double left_wheel_velocity_command_;
  double right_wheel_velocity_command_;
  
  // Previous encoder counts for velocity calculation
  int32_t prev_left_encoder_;
  int32_t prev_right_encoder_;
  rclcpp::Time last_read_time_;
  
  // Helper methods
  bool open_serial_port();
  void close_serial_port();
  bool send_command(const std::vector<uint8_t>& command);
  bool read_sensor_data(uint8_t packet_id, std::vector<uint8_t>& data);
  void start_oi();
  void set_mode_full();
  void update_odometry(const rclcpp::Duration& period);
  void send_wheel_velocities();
  int16_t bytes_to_int16(uint8_t high_byte, uint8_t low_byte);
  
  // OI Commands
  static constexpr uint8_t OI_START = 128;
  static constexpr uint8_t OI_FULL = 132;
  static constexpr uint8_t OI_DRIVE_DIRECT = 145;
  static constexpr uint8_t OI_SENSORS = 142;
  static constexpr uint8_t OI_QUERY_LIST = 149;
  
  // Sensor packet IDs
  static constexpr uint8_t PACKET_LEFT_ENCODER = 43;
  static constexpr uint8_t PACKET_RIGHT_ENCODER = 44;
};

}  // namespace create2_hardware

#endif  // CREATE2_HARDWARE_INTERFACE_HPP