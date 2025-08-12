#include "lerobot_hardware/so101_hardware.hpp"
#include "feetech_lib.hpp"

#include <limits>
#include <vector>
#include <unistd.h> // write(), read(), close()

#include "rclcpp/rclcpp.hpp"

namespace so101_hardware
{
    hardware_interface::CallbackReturn Create2HardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Get device path from hardware parameters
        device_path_ = info_.hardware_parameters["device"];
        baud_rate_ = std::stoi(info_.hardware_parameters.count("baud_rate") ? info_.hardware_parameters["baud_rate"] : "115200");

        std::vector<uint8_t> servo_ids = {0x0C};
        FeetechServo servo(port, baud, frequency, servo_ids, false);
        // Initialize state variables
        left_wheel_position_ = 0.0;
        right_wheel_position_ = 0.0;
        left_wheel_velocity_command_ = 0.0;
        right_wheel_velocity_command_ = 0.0;
        prev_left_encoder_ = 0;
        prev_right_encoder_ = 0;

        // Check that we have the expected joints
        if (info_.joints.size() != 2)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("Create2HardwareInterface"),
                "Expected exactly 2 joints, got %zu", info_.joints.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(
            rclcpp::get_logger("Create2HardwareInterface"),
            "Initialized Create2 hardware interface with device: %s", device_path_.c_str());

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Create2HardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[0].name, "position", &left_wheel_position_));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[1].name, "position", &right_wheel_position_));

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Create2HardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[0].name, "velocity", &left_wheel_velocity_command_));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[1].name, "velocity", &right_wheel_velocity_command_));

        return command_interfaces;
    }

    hardware_interface::CallbackReturn Create2HardwareInterface::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("Create2HardwareInterface"), "Activating...");

        if (!open_serial_port())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize Create 2
        start_oi();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        set_mode_full();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Read initial encoder values
        std::vector<uint8_t> left_data, right_data;
        if (read_sensor_data(PACKET_LEFT_ENCODER, left_data) && left_data.size() >= 2)
        {
            prev_left_encoder_ = bytes_to_int16(left_data[0], left_data[1]);
        }
        if (read_sensor_data(PACKET_RIGHT_ENCODER, right_data) && right_data.size() >= 2)
        {
            prev_right_encoder_ = bytes_to_int16(right_data[0], right_data[1]);
        }

        last_read_time_ = rclcpp::Clock().now();

        RCLCPP_INFO(rclcpp::get_logger("Create2HardwareInterface"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Create2HardwareInterface::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        RCLCPP_INFO(rclcpp::get_logger("Create2HardwareInterface"), "Deactivating...");

        // Stop the robot
        left_wheel_velocity_command_ = 0.0;
        right_wheel_velocity_command_ = 0.0;
        send_wheel_velocities();

        close_serial_port();

        RCLCPP_INFO(rclcpp::get_logger("Create2HardwareInterface"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Create2HardwareInterface::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // Read encoder data from Create 2
        std::vector<uint8_t> encoder_data;
        std::vector<uint8_t> query_cmd = {OI_QUERY_LIST, 2, PACKET_LEFT_ENCODER, PACKET_RIGHT_ENCODER};

        if (send_command(query_cmd))
        {
            encoder_data.resize(4); // 4 bytes of data
            const ssize_t bytes_read = ::read(serial_fd_, encoder_data.data(), encoder_data.size());

            if (bytes_read == 4)
            {
                // Parse encoder data
                const int32_t left_encoder = bytes_to_int16(encoder_data[0], encoder_data[1]);  // Skip packet ID
                const int32_t right_encoder = bytes_to_int16(encoder_data[2], encoder_data[3]); // Skip packet ID

                // Calculate position and velocity
                int32_t left_delta;
                int32_t right_delta;
                left_delta = (left_encoder - prev_left_encoder_) % (1 << 16);
                right_delta = (right_encoder - prev_right_encoder_) % (1 << 16);

                // Convert to radians
                const double left_wheel_delta = left_delta / counts_per_rev_ * 6.28;
                const double right_wheel_delta = right_delta / counts_per_rev_ * 6.28;

                // Update position
                left_wheel_position_ += left_wheel_delta;
                right_wheel_position_ += right_wheel_delta;

                prev_left_encoder_ = left_encoder;
                prev_right_encoder_ = right_encoder;
                // RCLCPP_INFO(rclcpp::get_logger("Create2HardwareInterface"), "Encoder positions: %d, %d", left_encoder, right_encoder);
                // RCLCPP_INFO(rclcpp::get_logger("Create2HardwareInterface"), "Wheel positions: %f, %f", left_wheel_position_, right_wheel_position_);
                return hardware_interface::return_type::OK;
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("Create2HardwareInterface"), "Bytes read from encoder query was not 6: %zu", bytes_read);
            }
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("Create2HardwareInterface"), "Encoder query command was not successful");
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Create2HardwareInterface::write(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        send_wheel_velocities();
        return hardware_interface::return_type::OK;
    }

    bool Create2HardwareInterface::open_serial_port()
    {
        serial_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Create2HardwareInterface"),
                         "Error opening serial port %s", device_path_.c_str());
            return false;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Create2HardwareInterface"),
                         "Error from tcgetattr");
            close(serial_fd_);
            return false;
        }

        // Set baud rate
        speed_t speed = (baud_rate_ == 19200) ? B19200 : B115200;
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // Set 8N1
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Create2HardwareInterface"),
                         "Error from tcsetattr");
            close(serial_fd_);
            return false;
        }

        return true;
    }

    void Create2HardwareInterface::close_serial_port()
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;
        }
    }

    bool Create2HardwareInterface::send_command(const std::vector<uint8_t> &command)
    {
        if (serial_fd_ < 0)
            return false;

        ssize_t bytes_written = ::write(serial_fd_, command.data(), command.size());
        return bytes_written == static_cast<ssize_t>(command.size());
    }

    bool Create2HardwareInterface::read_sensor_data(uint8_t packet_id, std::vector<uint8_t> &data)
    {
        std::vector<uint8_t> cmd = {OI_SENSORS, packet_id};
        if (!send_command(cmd))
            return false;

        // Determine expected data size based on packet ID
        size_t expected_size = 2; // Default for encoder packets
        data.resize(expected_size);

        ssize_t bytes_read = ::read(serial_fd_, data.data(), expected_size);
        return bytes_read == static_cast<ssize_t>(expected_size);
    }

    void Create2HardwareInterface::start_oi()
    {
        std::vector<uint8_t> cmd = {OI_START};
        send_command(cmd);
    }

    void Create2HardwareInterface::set_mode_full()
    {
        std::vector<uint8_t> cmd = {OI_FULL};
        send_command(cmd);
    }

    void Create2HardwareInterface::send_wheel_velocities()
    {
        // Convert from rad/s to mm/s
        double left_velocity_mms = left_wheel_velocity_command_ * wheel_radius_ * 1000.0;
        double right_velocity_mms = right_wheel_velocity_command_ * wheel_radius_ * 1000.0;

        // Clamp to Create 2 limits (-500 to 500 mm/s)
        left_velocity_mms = std::max(-500.0, std::min(500.0, left_velocity_mms));
        right_velocity_mms = std::max(-500.0, std::min(500.0, right_velocity_mms));

        // Convert to signed 16-bit integers
        int16_t right_vel = static_cast<int16_t>(right_velocity_mms);
        int16_t left_vel = static_cast<int16_t>(left_velocity_mms);

        // Create Drive Direct command
        std::vector<uint8_t> cmd = {
            OI_DRIVE_DIRECT,
            static_cast<uint8_t>(right_vel >> 8),   // Right velocity high byte
            static_cast<uint8_t>(right_vel & 0xFF), // Right velocity low byte
            static_cast<uint8_t>(left_vel >> 8),    // Left velocity high byte
            static_cast<uint8_t>(left_vel & 0xFF)   // Left velocity low byte
        };

        send_command(cmd);
    }

    int16_t Create2HardwareInterface::bytes_to_int16(uint8_t high_byte, uint8_t low_byte)
    {
        return static_cast<int16_t>(high_byte << 8) | low_byte;
    }

} // namespace create2_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    create2_hardware::Create2HardwareInterface, hardware_interface::SystemInterface)