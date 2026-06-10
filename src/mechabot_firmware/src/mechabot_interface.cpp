#include "mechabot_firmware/mechabot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sstream>
#include <iomanip>

namespace mechabot_firmware
{

MechabotInterface::MechabotInterface() {}

MechabotInterface::~MechabotInterface()
{
  if (esp_.IsOpen())
  {
    try { esp_.Close(); }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MechabotInterface"),
                          "Error closing port " << port_);
    }
  }
}

CallbackReturn MechabotInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
  auto result = hardware_interface::SystemInterface::on_init(hardware_info);
  if (result != CallbackReturn::SUCCESS) return result;

  try {
    port_ = info_.hardware_parameters.at("port");
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("MechabotInterface"), "No Serial Port provided!");
    return CallbackReturn::FAILURE;
  }

  velocity_commands_.resize(info_.joints.size(), 0.0);
  position_states_.resize(info_.joints.size(), 0.0);
  velocity_states_.resize(info_.joints.size(), 0.0);

  last_run_ = rclcpp::Clock().now();

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MechabotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);

    state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
  }

  // ✅ ADD BATTERY STATE
  state_interfaces.emplace_back(
      "battery", "percentage", &battery_percentage_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MechabotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
  }

  return command_interfaces;
}

CallbackReturn MechabotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MechabotInterface"), "Starting robot hardware ...");

  velocity_commands_ = {0.0, 0.0};
  position_states_ = {0.0, 0.0};
  velocity_states_ = {0.0, 0.0};

  try {
    esp_.Open(port_);
    esp_.SetBaudRate(LibSerial::BaudRate::BAUD_500000);
  } catch (...) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("MechabotInterface"),
                        "Error opening port " << port_);
    return CallbackReturn::FAILURE;
  }

  // ✅ INIT ROS NODE + PUBLISHER
  node_ = rclcpp::Node::make_shared("mechabot_battery_node");
  battery_pub_ = node_->create_publisher<std_msgs::msg::Float32>(
      "battery_percentage", 10);

  executor_.add_node(node_);

  RCLCPP_INFO(rclcpp::get_logger("MechabotInterface"),
              "Hardware started");

  return CallbackReturn::SUCCESS;
}

CallbackReturn MechabotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("MechabotInterface"), "Stopping robot hardware ...");

  if (esp_.IsOpen())
  {
    try { esp_.Close(); }
    catch (...)
    {
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("MechabotInterface"),
                          "Error closing port " << port_);
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MechabotInterface::read(const rclcpp::Time &,
                                                       const rclcpp::Duration &)
{
  if (esp_.IsDataAvailable())
  {
    auto dt = (rclcpp::Clock().now() - last_run_).seconds();
    std::string message;

    try
    {
      esp_.ReadLine(message, '\n', 100);

      if (!message.empty())
      {
        std::stringstream ss(message);
        std::string res;

        while (std::getline(ss, res, ','))
        {
          if (res.length() < 2) continue;

          // ✅ RIGHT WHEEL
          if (res.at(0) == 'r')
          {
            int multiplier = res.at(1) == 'p' ? 1 : -1;
            velocity_states_[0] = multiplier * std::stod(res.substr(2));
            position_states_[0] += velocity_states_[0] * dt;
          }

          // ✅ LEFT WHEEL
          else if (res.at(0) == 'l')
          {
            int multiplier = res.at(1) == 'p' ? 1 : -1;
            velocity_states_[1] = multiplier * std::stod(res.substr(2));
            position_states_[1] += velocity_states_[1] * dt;
          }

          // ✅ BATTERY PARSE
          else if (res.at(0) == 'b')
          {
            try {
              battery_percentage_ = std::stod(res.substr(1));
            } catch (...) {
              // ignore bad data
            }
          }
        }

        last_run_ = rclcpp::Clock().now();

        // ✅ PUBLISH BATTERY
        std_msgs::msg::Float32 msg;
        msg.data = battery_percentage_;
        battery_pub_->publish(msg);

        executor_.spin_some();
      }
    }
    catch (const std::exception &e)
    {
      static auto logger = rclcpp::get_logger("MechabotInterface");
      static auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
      RCLCPP_WARN_THROTTLE(logger, *clock, 1000,
                           "Serial read failed: %s", e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MechabotInterface::write(const rclcpp::Time &,
                                                        const rclcpp::Duration &)
{
  std::stringstream message_stream;

  char r_sign = velocity_commands_[0] >= 0 ? 'p' : 'n';
  char l_sign = velocity_commands_[1] >= 0 ? 'p' : 'n';

  message_stream << std::fixed << std::setprecision(2)
                 << "r" << r_sign << std::setw(5) << std::setfill('0') << std::abs(velocity_commands_[0])
                 << ",l" << l_sign << std::setw(5) << std::setfill('0') << std::abs(velocity_commands_[1])
                 << ",";

  try {
    esp_.Write(message_stream.str());
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("MechabotInterface"),
                        "Write failed: " << message_stream.str());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mechabot_firmware

PLUGINLIB_EXPORT_CLASS(mechabot_firmware::MechabotInterface,
                       hardware_interface::SystemInterface)