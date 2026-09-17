#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <cmath>
#include "so101_hardware/so101_system.hpp"

namespace so101_hardware {
    hardware_interface::CallbackReturn SO101SystemHardware::on_init(const hardware_interface::HardwareInfo& info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;
        
        SO101SystemHardware::hw_positions_.resize(info_.joints.size(),0.0);
        SO101SystemHardware::hw_commands_.resize(info_.joints.size(),0.0);

        baud_rate_ = std::stoi(info_.hardware_parameters["baud_rate"]);
        serial_port_ = info_.hardware_parameters["serial_port"];

        return CallbackReturn::SUCCESS;
    }
    
    hardware_interface::CallbackReturn SO101SystemHardware::on_configure( const rclcpp_lifecycle::State& previous_state)
    {
        if (!sms_sts_.begin(baud_rate_, serial_port_.c_str())) 
            return CallbackReturn::ERROR;
        return CallbackReturn::SUCCESS;
    }
    
    hardware_interface::CallbackReturn SO101SystemHardware::on_activate( const rclcpp_lifecycle::State& previous_state)
    {
        for ( size_t i = 0; i < info_.joints.size(); i++ )
        {
            sms_sts_.EnableTorque(i+1, 1);
        }
        return CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn SO101SystemHardware::on_deactivate( const rclcpp_lifecycle::State& previous_state)
    {
        for ( size_t i = 0; i < info_.joints.size(); i++ )
        {
            sms_sts_.EnableTorque(i+1, 0);
        }
        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> SO101SystemHardware::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for ( size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_positions_[i]
            );
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> SO101SystemHardware::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0 ; i < info_.joints.size(); i++ )
        {
            command_interfaces.emplace_back(
                info_.joints[i].name,
                hardware_interface::HW_IF_POSITION,
                &hw_commands_[i]
            );
        }
        return command_interfaces;
    }

    hardware_interface::return_type SO101SystemHardware::read( const rclcpp::Time& t, const rclcpp::Duration& p )
    {
        for(size_t i = 0 ; i < info_.joints.size(); i++)
        {
            int raw = sms_sts_.ReadPos(i+1);
            hw_positions_[i] = (raw-2048) *2 *M_PI / 4096.0;
        }
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type SO101SystemHardware::write( const rclcpp::Time& t, const rclcpp::Duration& p )
    {
        s16 pos[6];
        u16 speed[6];
        u8 acc[6];
        
        for ( size_t i = 0 ; i < info_.joints.size(); i++)
        {
            int raw = hw_commands_[i] * 4096/(2*M_PI) + 2048;
            pos[i] = raw;    
            speed[i] = 1000;
            acc[i] = 50;
        }
        sms_sts_.SyncWritePosEx( ids_, 6, pos, speed, acc );
        return hardware_interface::return_type::OK;
    }
}

PLUGINLIB_EXPORT_CLASS(so101_hardware::SO101SystemHardware, hardware_interface::SystemInterface)