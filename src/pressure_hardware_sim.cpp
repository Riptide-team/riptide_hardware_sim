#include "riptide_hardware_sim/pressure_hardware_sim.hpp"

#include <memory>
#include <string>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/transport/NodeOptions.hh>

#include <ignition/msgs/altimeter.pb.h>


namespace riptide_hardware_sim {

    CallbackReturn PressureHardwareSim::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

        if (info_.hardware_parameters.find("altimeter_topic") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("PressureHardwareSim"),
                "You need to specify the altimeter_topic in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardwareSim"), "pressure_topic: %s", (info_.hardware_parameters["altimeter_topic"]).c_str());

        ignition::transport::NodeOptions node_option;
        node = std::make_shared<ignition::transport::Node>(node_option);

        node->Subscribe(info_.hardware_parameters["altimeter_topic"], &PressureHardwareSim::callback, this);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> PressureHardwareSim::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // export sensor state interface
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
        }

        return state_interfaces;
    }

    CallbackReturn PressureHardwareSim::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardwareSim"), "Activating ...please wait...");

        // Set joint state
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); ++i) {
            if (std::isnan(hw_sensor_states_[i]))
                hw_sensor_states_[i] = 0;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardwareSim"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn PressureHardwareSim::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardwareSim"), "Deactivating ...please wait...");

        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardwareSim"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void PressureHardwareSim::callback(const ignition::msgs::Altimeter & msg) {
        std::scoped_lock<std::mutex> lock(mutex_);
        depth_feedback_ = msg.vertical_position();
    }

    hardware_interface::return_type PressureHardwareSim::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Locking mutex
        std::scoped_lock<std::mutex> lock(mutex_);

        // Set joint state
        hw_sensor_states_[0] = depth_feedback_;

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware_sim


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware_sim::PressureHardwareSim, hardware_interface::SensorInterface)