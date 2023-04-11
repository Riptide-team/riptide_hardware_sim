#include "riptide_hardware_sim/imu_hardware_sim.hpp"

#include <memory>
#include <string>

#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/transport/NodeOptions.hh>


namespace riptide_hardware_sim {

    CallbackReturn IMUHardwareSim::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        hw_sensor_states_.resize(info_.sensors[0].state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());

        namespace_ = info_.hardware_parameters["namespace"];

        imu_topic = info_.hardware_parameters["imu_topic"];

        RCLCPP_DEBUG(rclcpp::get_logger("IMUHardwareSim"), "namespace: %s, imu_topic: %s", namespace_.c_str(), imu_topic.c_str());

        ignition::transport::NodeOptions node_option;
        node_option.SetNameSpace(namespace_);
        node = std::make_shared<ignition::transport::Node>(node_option);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> IMUHardwareSim::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // export sensor state interface
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &hw_sensor_states_[i]));
        }

        return state_interfaces;
    }

    CallbackReturn IMUHardwareSim::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("IMUHardwareSim"), "Activating ...please wait...");

        // Set joint state
        for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); ++i) {
            if (std::isnan(hw_sensor_states_[i]))
                hw_sensor_states_[i] = 0;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("IMUHardwareSim"), "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn IMUHardwareSim::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("IMUHardwareSim"), "Deactivating ...please wait...");

        RCLCPP_DEBUG(rclcpp::get_logger("IMUHardwareSim"), "Successfully deactivated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type IMUHardwareSim::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Request imu infos from the simulator
        ignition::msgs::IMU rep;
        bool result;
        unsigned int timeout = 5000;

        // Request the imu service.
        bool executed = node->Request(imu_topic, timeout, rep, result);

        if (!executed) {
            RCLCPP_FATAL(rclcpp::get_logger("IMUHardwareSim"), "Service call timed out");
            return hardware_interface::return_type::ERROR;
        }

        if (!result) {
            RCLCPP_FATAL(rclcpp::get_logger("IMUHardwareSim"), "Service call failed");
            return hardware_interface::return_type::ERROR;
        }

        // Set joint state
        hw_sensor_states_[0] = rep.linear_acceleration().x();
        hw_sensor_states_[1] = rep.linear_acceleration().y();
        hw_sensor_states_[2] = rep.linear_acceleration().z();

        hw_sensor_states_[3] = rep.angular_velocity().x();
        hw_sensor_states_[4] = rep.angular_velocity().y();
        hw_sensor_states_[5] = rep.angular_velocity().z();

        hw_sensor_states_[6] = rep.orientation().x();
        hw_sensor_states_[7] = rep.orientation().y();
        hw_sensor_states_[8] = rep.orientation().z();
        hw_sensor_states_[9] = rep.orientation().w();

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware_sim


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware_sim::IMUHardwareSim, hardware_interface::SensorInterface)