/**
 * ============================================================================
 * ArmSystemInterface — ros2_control SystemInterface for MyActuator RMD
 * ============================================================================
 *
 * ArmDriver (pure C++) 를 has-a 멤버로 사용하는 얇은 ROS2 브리지.
 * Lifecycle 콜백에서 ArmDriver 를 configure/activate/deactivate 하고,
 * read()/write() 에서 ArmDriver 의 readAll()/writeAll() 을 호출.
 *
 * ============================================================================
 */

#ifndef ARM_SDK_ROS2__ARM_SYSTEM_INTERFACE_HPP_
#define ARM_SDK_ROS2__ARM_SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <chrono>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "arm_sdk/arm_driver.hpp"

namespace arm_sdk_ros2
{

class ArmSystemInterface : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(ArmSystemInterface)

    // Lifecycle callbacks
    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(
        const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_error(
        const rclcpp_lifecycle::State & previous_state) override;

    // Interface exports
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    // Control loop
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    static constexpr const char* LOGGER = "ArmSystemInterface";

    // Native driver (pure C++)
    std::unique_ptr<arm_sdk::ArmDriver> driver_;

    // Parsed from HardwareInfo
    arm_sdk::ArmDriverConfig driver_config_;
    bool auto_home_ = false;
    std::vector<double> ready_positions_rad_;

    // Timing diagnostics
    double last_read_time_ms_  = 0.0;
    double last_write_time_ms_ = 0.0;
    double avg_read_time_ms_   = 0.0;
    double avg_write_time_ms_  = 0.0;
    uint64_t comm_count_ = 0;
    bool enable_timing_log_ = false;
};

}  // namespace arm_sdk_ros2

#endif  // ARM_SDK_ROS2__ARM_SYSTEM_INTERFACE_HPP_
