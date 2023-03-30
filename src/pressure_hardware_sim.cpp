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

#include "riptide_simulator/msgs/pressuremsg.pb.h"


namespace riptide_hardware_sim {

    CallbackReturn PressureHardwareSim::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

        namespace_ = info_.hardware_parameters["namespace"];

        pressure_topic = info_.hardware_parameters["pressure_topic"];

        RCLCPP_DEBUG(rclcpp::get_logger("PressureHardwareSim"), "namespace: %s, pressure_topic: %s", namespace_.c_str(), pressure_topic.c_str());

        ignition::transport::NodeOptions node_option;
        node_option.SetNameSpace(namespace_);
        node = std::make_shared<ignition::transport::Node>(node_option);

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

    hardware_interface::return_type PressureHardwareSim::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Request pressure from the simulator
        riptide::msgs::PressureMsg rep;
        bool result;
        unsigned int timeout = 5000;

        // Request the imu service.
        bool executed = node->Request(pressure_topic, timeout, rep, result);

        if (!executed) {
            RCLCPP_FATAL(rclcpp::get_logger("PressureHardwareSim"), "Service call timed out");
            return hardware_interface::return_type::ERROR;
        }

        if (!result) {
            RCLCPP_FATAL(rclcpp::get_logger("PressureHardwareSim"), "Service call failed");
            return hardware_interface::return_type::ERROR;
        }

        // Set joint state
        hw_sensor_states_[0] = rep.pressure();
        hw_sensor_states_[1] = rep.temperature();
        hw_sensor_states_[2] = rep.depth();
        hw_sensor_states_[3] = rep.altitude();

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware_sim


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware_sim::PressureHardwareSim, hardware_interface::SensorInterface)