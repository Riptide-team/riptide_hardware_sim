#include "riptide_hardware_sim/actuators_hardware_sim.hpp"

#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "riptide_simulator/msgs/controlmsg.pb.h"
#include "riptide_simulator/msgs/multijointmsg.pb.h"
#include <ignition/msgs.hh>
#include <ignition/transport.hh>


namespace riptide_hardware_sim {

    CallbackReturn ActuatorsHardwareSim::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Checking control and state filename
        if (info_.hardware_parameters.find("control_topic") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ActuatorsHardwareSim"),
                "You need to specify the control_topic in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info_.hardware_parameters.find("state_topic") == info_.hardware_parameters.end()) {
            RCLCPP_FATAL(
                rclcpp::get_logger("ActuatorsHardwareSim"),
                "You need to specify the state_topic in ros2_control urdf tag as param!"
            );
            return hardware_interface::CallbackReturn::ERROR;
        }

        namespace_ = info_.hardware_parameters["namespace"];
        control_topic = info_.hardware_parameters["control_topic"];
        state_topic = info_.hardware_parameters["state_topic"];

        hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
        hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

        RCLCPP_DEBUG(
            rclcpp::get_logger("ActuatorsHardwareSim"),
            "control_topic: %s",
            control_topic.c_str()
        );

        RCLCPP_DEBUG(
            rclcpp::get_logger("ActuatorsHardwareSim"),
            "state_topic: %s",
            state_topic.c_str()
        );

        ignition::transport::NodeOptions node_option;
        node_option.SetNameSpace(namespace_);
        node = std::make_shared<ignition::transport::Node>(node_option);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ActuatorsHardwareSim::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // export sensor state interface
        for (uint i = 0; i < info_.joints.size(); ++i) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_states_positions_[i]));
        }

        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ActuatorsHardwareSim::export_command_interfaces() {
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // export command state interface
        for (uint i = 0; i < info_.joints.size(); ++i) {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, info_.joints[i].state_interfaces[0].name, &hw_commands_positions_[i]));
        }

        return command_interfaces;
    }

    CallbackReturn ActuatorsHardwareSim::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("ActuatorsHardwareSim"), "Activating ...please wait...");
        for (unsigned int i=0; i<hw_states_positions_.size(); ++i) {
            hw_states_positions_[i] = 0;
        }
        for (unsigned int i=0; i<hw_commands_positions_.size(); ++i) {
            hw_commands_positions_[i] = 0;
        }
        RCLCPP_DEBUG(rclcpp::get_logger("ActuatorsHardwareSim"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn ActuatorsHardwareSim::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("ActuatorsHardwareSim"), "Deactivating ...please wait...");
        RCLCPP_DEBUG(rclcpp::get_logger("ActuatorsHardwareSim"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type ActuatorsHardwareSim::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Call service to set control
        riptide::msgs::MultiJointMsg rep;

        // Request the state_topic service.
        bool result;
        unsigned int timeout = 10000;

        // Request the "/quote" service.
        bool executed = node->Request(state_topic, timeout, rep, result);

        if (!executed) {
            RCLCPP_FATAL(rclcpp::get_logger("ActuatorHardwareSim"), "Service call failed [%s]", state_topic.c_str());
            return hardware_interface::return_type::ERROR;
        }

        if (!result) {
            RCLCPP_FATAL(rclcpp::get_logger("ActuatorHardwareSim"), "Service call timed out");
            return hardware_interface::return_type::ERROR;
        }

        for (int i=0; i<rep.joint_size(); ++i) {
            hw_states_positions_[i] = rep.joint(i).position();
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ActuatorsHardwareSim::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Call service to set control
        riptide::msgs::ControlMsg req;
        req.set_thrust(hw_commands_positions_[0]);
        req.set_d(hw_commands_positions_[1]);
        req.set_p(hw_commands_positions_[2]);
        req.set_s(hw_commands_positions_[3]);

        // RCLCPP_INFO(rclcpp::get_logger("ActuatorsHardwareSim"), "Writing %f %f %f %f", hw_commands_positions_[0],
        // hw_commands_positions_[1], hw_commands_positions_[2], hw_commands_positions_[3]);

        // Request the control_topic service.
        bool executed = node->Request(control_topic, req);

        if (!executed) {
            RCLCPP_FATAL(rclcpp::get_logger("ActuatorHardwareSim"), "Service call failed [%s]", control_topic.c_str());
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }
} // riptide_hardware_sim


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware_sim::ActuatorsHardwareSim, hardware_interface::SystemInterface)