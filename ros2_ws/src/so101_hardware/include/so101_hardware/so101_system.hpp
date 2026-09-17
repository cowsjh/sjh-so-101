#ifndef SO101_HARDWARE__SO101_SYSTEM_HPP_
#define SO101_HARDWARE__SO101_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <vector>
#include "scservo/SMS_STS.h"

namespace so101_hardware {
    class SO101SystemHardware : public hardware_interface::SystemInterface
    {
    public:
        hardware_interface::CallbackReturn on_init (const hardware_interface::HardwareInfo& info) override;
        hardware_interface::CallbackReturn on_configure( const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate( const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate( const rclcpp_lifecycle::State& previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read ( const rclcpp::Time& t, const rclcpp::Duration& p ) override;
        hardware_interface::return_type write ( const rclcpp::Time& t, const rclcpp::Duration& p ) override;
    private:
        std::vector<double> hw_positions_;
        std::vector<double> hw_commands_;

        SMS_STS sms_sts_{0};
        std::string serial_port_;
        int baud_rate_;
        u8 ids_[6] = {1,2,3,4,5,6};
    };
}

#endif