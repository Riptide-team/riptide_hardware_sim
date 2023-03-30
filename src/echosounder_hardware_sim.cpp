#include "riptide_hardware_sim/echosounder_hardware_sim.hpp"

#include <string>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"

#include "riptide_simulator/msgs/controlmsg.pb.h"
#include "riptide_simulator/msgs/multijointmsg.pb.h"
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <ignition/transport/NodeOptions.hh>


namespace riptide_hardware_sim {

    CallbackReturn EchoSounderHardwareSim::on_init(const hardware_interface::HardwareInfo & info) {
        if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        state_topic_ = "echosounder";

        namespace_ = info_.hardware_parameters["namespace"];

        state_topic_ = info_.hardware_parameters["state_topic"];

        range_ = std::numeric_limits<double>::quiet_NaN();

        RCLCPP_DEBUG(
            rclcpp::get_logger("EchoSounderHardwareSim"),
            "namespace: %s, state topic: %s",
            namespace_.c_str(),
            state_topic_.c_str()
        );

        ignition::transport::NodeOptions node_option;
        node_option.SetNameSpace(namespace_);
        node = std::make_shared<ignition::transport::Node>(node_option);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> EchoSounderHardwareSim::export_state_interfaces() {
        std::vector<hardware_interface::StateInterface> state_interfaces;

        // export sensor state interface
        state_interfaces.emplace_back(
            hardware_interface::StateInterface(
                info_.sensors[0].name, info_.sensors[0].state_interfaces[0].name, &range_
            )
        );

        return state_interfaces;
    }

    CallbackReturn EchoSounderHardwareSim::on_activate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("EchoSounderHardwareSim"), "Activating ...please wait...");
        range_ = 0;
        RCLCPP_DEBUG(rclcpp::get_logger("EchoSounderHardwareSim"), "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    CallbackReturn EchoSounderHardwareSim::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/) {
        RCLCPP_DEBUG(rclcpp::get_logger("EchoSounderHardwareSim"), "Deactivating ...please wait...");
        RCLCPP_DEBUG(rclcpp::get_logger("EchoSounderHardwareSim"), "Successfully deactivated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type EchoSounderHardwareSim::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        // Call service to set control
        ignition::msgs::Float rep;

        // Request the state_topic service.
        bool result;
        unsigned int timeout = 5000;

        // Request the service.
        bool executed = node->Request(state_topic_, timeout, rep, result);

        if (!executed) {
            RCLCPP_FATAL(rclcpp::get_logger("EchoSounderHardwareSim"), "Service call failed [%s]", state_topic_.c_str());
            return hardware_interface::return_type::ERROR;
        }

        if (!result) {
            RCLCPP_FATAL(rclcpp::get_logger("EchoSounderHardwareSim"), "Service call timed out");
            return hardware_interface::return_type::ERROR;
        }

        range_ = rep.data();

        return hardware_interface::return_type::OK;
    }

} // riptide_hardware_sim


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(riptide_hardware_sim::EchoSounderHardwareSim, hardware_interface::SensorInterface)