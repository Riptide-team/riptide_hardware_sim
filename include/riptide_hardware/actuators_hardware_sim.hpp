#ifndef ACTUATORS_HARDWARE_SIM_HPP
#define ACTUATORS_HARDWARE_SIM_HPP

#include <memory>
#include <string>

#include <ignition/transport.hh>
#include <ignition/transport/NodeOptions.hh>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


namespace riptide_hardware {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class ActuatorsHardwareSim : public hardware_interface::SystemInterface {

        public:
            ActuatorsHardwareSim() {};

            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces()override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

            hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

        private:
            std::vector<double> hw_states_positions_;
            std::vector<double> hw_commands_positions_;

            // Ignition node
            std::shared_ptr<ignition::transport::Node> node;

            // Namespace
            std::string namespace_;

            // Control topic
            std::string control_topic;

            // Control topic
            std::string state_topic;
    };
} // riptide_hardware

#endif // ACTUATORS_HARDWARE_SIM_HPP