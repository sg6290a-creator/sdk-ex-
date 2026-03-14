/**
 * ============================================================================
 * ArmSystemInterface — Implementation
 * ============================================================================
 *
 * HardwareInfo → ArmDriverConfig 파싱,
 * ArmDriver lifecycle 호출, 자동 홈/레디 시퀀스 관리.
 *
 * ============================================================================
 */

#include "arm_sdk_ros2/arm_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace arm_sdk_ros2
{

// ════════════════════════════════════════════════════════════════
//  Helper: formatted logging boxes
// ════════════════════════════════════════════════════════════════
static void printErrorBox(rclcpp::Logger logger, const std::string& title,
                          const std::vector<std::string>& lines)
{
    RCLCPP_ERROR(logger, " ");
    RCLCPP_ERROR(logger, "╔══════════════════════════════════════════════════════════════╗");
    RCLCPP_ERROR(logger, "║  %-60s║", title.c_str());
    RCLCPP_ERROR(logger, "╠══════════════════════════════════════════════════════════════╣");
    for (const auto& line : lines) {
        RCLCPP_ERROR(logger, "║  %-60s║", line.c_str());
    }
    RCLCPP_ERROR(logger, "╚══════════════════════════════════════════════════════════════╝");
    RCLCPP_ERROR(logger, " ");
}

static void printInfoBox(rclcpp::Logger logger, const std::string& title,
                         const std::vector<std::string>& lines)
{
    RCLCPP_INFO(logger, " ");
    RCLCPP_INFO(logger, "┌──────────────────────────────────────────────────────────────┐");
    RCLCPP_INFO(logger, "│  %-60s│", title.c_str());
    RCLCPP_INFO(logger, "├──────────────────────────────────────────────────────────────┤");
    for (const auto& line : lines) {
        RCLCPP_INFO(logger, "│  %-60s│", line.c_str());
    }
    RCLCPP_INFO(logger, "└──────────────────────────────────────────────────────────────┘");
    RCLCPP_INFO(logger, " ");
}

// ════════════════════════════════════════════════════════════════
//  on_init — Parse HardwareInfo → ArmDriverConfig
// ════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArmSystemInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    auto logger = rclcpp::get_logger(LOGGER);

    // Parse hardware parameters
    auto param = [&](const std::string& key, const std::string& def = "") -> std::string {
        auto it = info_.hardware_parameters.find(key);
        return (it != info_.hardware_parameters.end()) ? it->second : def;
    };

    // USBCAN device settings
    driver_config_.can_channel  = std::stoi(param("can_channel", "0"));
    driver_config_.lib_path     = param("lib_path", "libusbcan.so");

    // Baudrate → stored as Hz in config
    int baudrate = std::stoi(param("baudrate", "1000000"));
    driver_config_.baudrate = baudrate;

    // Motor dynamics
    driver_config_.motor_acceleration  = std::stoul(param("motor_acceleration", "500"));
    driver_config_.motor_deceleration  = std::stoul(param("motor_deceleration", "500"));
    driver_config_.max_velocity_dps    = std::stod(param("max_velocity", "360"));
    driver_config_.default_velocity_dps = std::stod(param("default_velocity", "50"));

    // Auto-home
    std::string auto_home_str = param("auto_home", "false");
    auto_home_ = (auto_home_str == "true" || auto_home_str == "True" || auto_home_str == "1");

    // Timing log
    std::string log_str = param("enable_timing_log", "false");
    enable_timing_log_ = (log_str == "true" || log_str == "True" || log_str == "1");

    RCLCPP_INFO(logger, "Motor dynamics: accel=%u dps², max_vel=%.0f dps, default_vel=%.0f dps",
                driver_config_.motor_acceleration, driver_config_.max_velocity_dps,
                driver_config_.default_velocity_dps);

    // Parse joints → ArmDriverConfig::JointDef
    for (const auto & joint : info_.joints) {
        auto jit = joint.parameters.find("actuator_id");
        if (jit == joint.parameters.end()) continue;

        arm_sdk::ArmDriverConfig::JointDef jd;
        jd.name        = joint.name;
        jd.actuator_id = std::stoi(jit->second);

        auto kt = joint.parameters.find("torque_constant");
        if (kt != joint.parameters.end()) {
            jd.torque_constant = std::stod(kt->second);
        }

        driver_config_.joints.push_back(jd);

        RCLCPP_INFO(logger, "  → Joint[%zu]: '%s' → Motor ID=%d, Kt=%.3f",
                     driver_config_.joints.size() - 1, jd.name.c_str(),
                     jd.actuator_id, jd.torque_constant);
    }

    if (driver_config_.joints.empty()) {
        printErrorBox(logger, "CONFIGURATION ERROR",
                      {"No joints with 'actuator_id' found in HardwareInfo",
                       "",
                       "Add to your URDF joint definition:",
                       "  <param name=\"actuator_id\">1</param>"});
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Ready positions for singularity avoidance (from SRDF)
    ready_positions_rad_ = {0.0, -0.69813, -2.35619, 0.05236};

    RCLCPP_INFO(logger, "✓ Configuration parsed: %zu motors via USBCAN",
                driver_config_.joints.size());
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════
//  on_configure — Create and configure ArmDriver
// ════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArmSystemInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto logger = rclcpp::get_logger(LOGGER);

    RCLCPP_INFO(logger, " ");
    RCLCPP_INFO(logger, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(logger, "  Configuring ArmSystemInterface (USBCAN + RMD)");
    RCLCPP_INFO(logger, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    driver_ = std::make_unique<arm_sdk::ArmDriver>();

    // Forward log to ROS
    driver_->setLogCallback([logger](int level, const std::string& msg) {
        switch (level) {
            case arm_sdk::LOG_ERROR: RCLCPP_ERROR(logger, "%s", msg.c_str()); break;
            case arm_sdk::LOG_WARN:  RCLCPP_WARN(logger, "%s", msg.c_str()); break;
            default:                 RCLCPP_INFO(logger, "%s", msg.c_str()); break;
        }
    });

    if (!driver_->configure(driver_config_)) {
        printErrorBox(logger, "DRIVER CONFIGURATION FAILED",
                      {"ArmDriver::configure() returned false",
                       "",
                       "Check:",
                       "  □ USBCAN device connected (lsusb | grep 0471)",
                       "  □ libusbcan.so installed (/lib/ or LD_LIBRARY_PATH)",
                       "  □ Motor power ON, CAN wiring correct"});
        driver_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }

    printInfoBox(logger, "CONFIGURATION SUCCESSFUL",
                 {"Motors: " + std::to_string(driver_->jointCount()) + " connected via USBCAN",
                  "CAN Channel: " + std::to_string(driver_config_.can_channel),
                  "Baudrate: " + std::to_string(driver_config_.baudrate)});

    return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════
//  on_activate — Read positions, auto-home, ready position
// ════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArmSystemInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto logger = rclcpp::get_logger(LOGGER);

    RCLCPP_INFO(logger, " ");
    RCLCPP_INFO(logger, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(logger, "  Activating ArmSystemInterface");
    RCLCPP_INFO(logger, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    driver_->stopHoldingThread();

    // Read current positions
    RCLCPP_INFO(logger, "[1/3] Reading current positions...");
    driver_->activate();

    for (size_t i = 0; i < driver_->jointCount(); ++i) {
        auto& j = driver_->joint(i);
        RCLCPP_INFO(logger, "  → '%s': %.3f rad (%.1f deg)",
                     j.joint_name.c_str(), j.position_state,
                     arm_sdk::RMDProtocol::radToDeg(j.position_state));
    }

    // Auto-home if enabled
    if (auto_home_) {
        RCLCPP_INFO(logger, "[2/3] Moving to home position...");
        for (size_t i = 0; i < driver_->jointCount(); ++i) {
            auto& j = driver_->joint(i);
            j.position_command = 0.0;
            j.velocity_command = arm_sdk::RMDProtocol::dpsToRadS(50.0);
            driver_->writeMotor(i);
        }

        // Wait for motors to reach home
        bool all_home = false;
        for (int iter = 0; iter < 100 && !all_home; ++iter) {
            all_home = true;
            for (size_t i = 0; i < driver_->jointCount(); ++i) {
                driver_->readMotor(i);
                if (std::abs(driver_->joint(i).position_state) > 0.05) {
                    all_home = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(logger, "  ✓ Motors at home position");

        std::this_thread::sleep_for(std::chrono::seconds(1));

        // Move to ready position (singularity avoidance)
        RCLCPP_INFO(logger, "[2.5/3] Moving to ready position...");
        for (size_t i = 0; i < driver_->jointCount() && i < ready_positions_rad_.size(); ++i) {
            auto& j = driver_->joint(i);
            j.position_command = ready_positions_rad_[i];
            j.velocity_command = arm_sdk::RMDProtocol::dpsToRadS(50.0);
            driver_->writeMotor(i);
            RCLCPP_INFO(logger, "  → '%s': moving to %.3f rad (%.1f deg)",
                         j.joint_name.c_str(), ready_positions_rad_[i],
                         arm_sdk::RMDProtocol::radToDeg(ready_positions_rad_[i]));
        }

        bool all_ready = false;
        for (int iter = 0; iter < 100 && !all_ready; ++iter) {
            all_ready = true;
            for (size_t i = 0; i < driver_->jointCount() && i < ready_positions_rad_.size(); ++i) {
                driver_->readMotor(i);
                if (std::abs(driver_->joint(i).position_state - ready_positions_rad_[i]) > 0.05) {
                    all_ready = false;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO(logger, "  ✓ Motors at ready position");
    } else {
        RCLCPP_INFO(logger, "[2/3] Auto-home disabled");
    }

    // Initialize commands to current positions
    RCLCPP_INFO(logger, "[3/3] Initializing command values...");
    for (size_t i = 0; i < driver_->jointCount(); ++i) {
        auto& j = driver_->joint(i);
        j.position_command = j.position_state;
        j.velocity_command = 0.0;
        j.effort_command   = 0.0;
    }

    RCLCPP_INFO(logger, "✓ Activation complete");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════
//  on_deactivate — Home + holding thread
// ════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArmSystemInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto logger = rclcpp::get_logger(LOGGER);
    RCLCPP_INFO(logger, "Deactivating...");

    if (auto_home_) {
        RCLCPP_INFO(logger, "Moving to home position...");
        for (size_t i = 0; i < driver_->jointCount(); ++i) {
            auto& j = driver_->joint(i);
            j.position_command = 0.0;
            j.velocity_command = arm_sdk::RMDProtocol::dpsToRadS(100.0);
            driver_->writeMotor(i);
        }
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    driver_->stopHoldingThread();
    driver_->startHoldingThread();

    RCLCPP_INFO(logger, "✓ Deactivation complete");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════
//  on_cleanup / on_error
// ════════════════════════════════════════════════════════════════
hardware_interface::CallbackReturn ArmSystemInterface::on_cleanup(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto logger = rclcpp::get_logger(LOGGER);
    RCLCPP_INFO(logger, "Cleaning up...");

    if (driver_) {
        driver_->cleanup();
        driver_.reset();
    }

    RCLCPP_INFO(logger, "✓ Cleanup complete");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmSystemInterface::on_error(
    const rclcpp_lifecycle::State & /*previous_state*/)
{
    auto logger = rclcpp::get_logger(LOGGER);
    RCLCPP_ERROR(logger, "Hardware error occurred!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ════════════════════════════════════════════════════════════════
//  Interface exports
// ════════════════════════════════════════════════════════════════
std::vector<hardware_interface::StateInterface>
ArmSystemInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;

    for (size_t i = 0; i < driver_->jointCount(); ++i) {
        auto& j = driver_->joint(i);
        interfaces.emplace_back(j.joint_name, hardware_interface::HW_IF_POSITION, &j.position_state);
        interfaces.emplace_back(j.joint_name, hardware_interface::HW_IF_VELOCITY, &j.velocity_state);
        interfaces.emplace_back(j.joint_name, hardware_interface::HW_IF_EFFORT, &j.effort_state);
    }

    return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ArmSystemInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;

    for (size_t i = 0; i < driver_->jointCount(); ++i) {
        auto& j = driver_->joint(i);
        interfaces.emplace_back(j.joint_name, hardware_interface::HW_IF_POSITION, &j.position_command);
        interfaces.emplace_back(j.joint_name, hardware_interface::HW_IF_VELOCITY, &j.velocity_command);
        interfaces.emplace_back(j.joint_name, hardware_interface::HW_IF_EFFORT, &j.effort_command);
    }

    return interfaces;
}

// ════════════════════════════════════════════════════════════════
//  read / write
// ════════════════════════════════════════════════════════════════
hardware_interface::return_type ArmSystemInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(driver_->mutex());

    auto start = std::chrono::high_resolution_clock::now();
    driver_->readAll();
    auto end = std::chrono::high_resolution_clock::now();

    last_read_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
    comm_count_++;
    avg_read_time_ms_ += (last_read_time_ms_ - avg_read_time_ms_) / comm_count_;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmSystemInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    std::lock_guard<std::mutex> lock(driver_->mutex());

    auto start = std::chrono::high_resolution_clock::now();
    driver_->writeAll();
    auto end = std::chrono::high_resolution_clock::now();

    last_write_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
    avg_write_time_ms_ += (last_write_time_ms_ - avg_write_time_ms_) / comm_count_;

    return hardware_interface::return_type::OK;
}

}  // namespace arm_sdk_ros2

// ════════════════════════════════════════════════════════════════
//  Plugin registration
// ════════════════════════════════════════════════════════════════
PLUGINLIB_EXPORT_CLASS(
    arm_sdk_ros2::ArmSystemInterface,
    hardware_interface::SystemInterface)
