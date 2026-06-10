#ifndef MECHABOT_INTERFACE_HPP
#define MECHABOT_INTERFACE_HPP

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

#include <std_msgs/msg/float32.hpp>   // ✅ NEW

#include <vector>
#include <string>

namespace mechabot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MechabotInterface : public hardware_interface::SystemInterface
{
public:
  MechabotInterface();
  virtual ~MechabotInterface();

  // Lifecycle
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // SystemInterface
  CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // Serial
  LibSerial::SerialPort esp_;
  std::string port_;

  // Robot data
  std::vector<double> velocity_commands_;
  std::vector<double> position_states_;
  std::vector<double> velocity_states_;

  // ✅ Battery state
  double battery_percentage_ = 0.0;

  // ✅ ROS publisher stuff
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
  rclcpp::executors::SingleThreadedExecutor executor_;

  // Timing
  rclcpp::Time last_run_;
};

}  // namespace mechabot_firmware

#endif  // MECHABOT_INTERFACE_HPP