#pragma once

#include <string>
#include <memory>

#include <ignition/transport.hh>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


namespace riptide_hardware_sim {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class BatteryHardwareSim : public hardware_interface::SensorInterface {

        public:
            BatteryHardwareSim() {};

            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::vector<double> hw_sensor_states_;

            // Ignition node
            std::shared_ptr<ignition::transport::Node> node;

            // Namespace
            std::string namespace_;

            // Battery topic
            std::string battery_topic;
    };
} // riptide_hardware