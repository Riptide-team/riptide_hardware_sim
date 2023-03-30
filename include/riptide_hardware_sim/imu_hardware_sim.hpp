#pragma once

#include <string>
#include <memory>

#include <ignition/transport.hh>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"


namespace riptide_hardware_sim {

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class IMUHardwareSim : public hardware_interface::SensorInterface {

        public:
            IMUHardwareSim() {};

            CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

            CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

            hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

        private:
            std::vector<double> hw_sensor_states_;

            // Ignition node
            std::shared_ptr<ignition::transport::Node> node;

            // Imu topic
            std::string namespace_;

            // Imu topic
            std::string imu_topic;
    };
} // riptide_hardware_sim