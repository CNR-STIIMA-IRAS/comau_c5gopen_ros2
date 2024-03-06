/*
 *  Software License Agreement (New BSD License)
 *
 *  Copyright 2024 National Council of Research of Italy (CNR)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __COMAU_MQTT_HW_CLIENT__
#define __COMAU_MQTT_HW_CLIENT__

#include <string>
#include <vector>

#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <comau_c5gopen_ros2/mqtt_driver.hpp>

using hardware_interface::return_type;

namespace cnr
{
  namespace comau
  {
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    static constexpr size_t POSITION_INTERFACE_INDEX = 0;
    static constexpr size_t VELOCITY_INTERFACE_INDEX = 1;

    class JointComms : public rclcpp::Node
    {
      public:
        JointComms();

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fb_pub_;
      private:
    };



    class HARDWARE_INTERFACE_PUBLIC ComauC5GopenHw : public hardware_interface::SystemInterface
    {
    public:
      CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

      std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

      std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

      return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

      return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

    protected:

      const std::vector<std::string> standard_interfaces_ = {
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT}; // CHECK TO KEEP ALL OR NOT!

      std::vector<double> joint_position_command_;
      std::vector<double> joint_positions_;
      std::vector<double> joint_velocities_;

    private:
      rclcpp::Logger logger_ = rclcpp::get_logger("comau_c5gopen_hw");
      rclcpp::executors::SingleThreadedExecutor executor_;
      std::shared_ptr<JointComms> comms_;
      std::vector<std::string> joint_names_; 
      std::vector<double>      joint_pos_;

      std::unique_ptr<cnr::comau::MQTTComauClient> mqtt_client_;
      std::string topic_fdb_pos_name_;
      std::string topic_fdb_vel_name_;
      std::string topic_cmd_name_;
      int mqtt_loop_timeout_; 

      bool read_only_;

      template <typename HandleType>
      bool get_interface(
        const std::string & name, const std::vector<std::string> & interface_list,
        const std::string & interface_name, const size_t vector_index,
        std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces);

      void initialize_storage_vectors(
        std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
        const std::vector<std::string> & interfaces,
        const std::vector<hardware_interface::ComponentInfo> & component_infos);

      template <typename InterfaceType>
      bool populate_interfaces(
        const std::vector<hardware_interface::ComponentInfo> & components,
        std::vector<std::string> & interfaces, std::vector<std::vector<double>> & storage,
        std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces);

    };

    typedef ComauC5GopenHw GenericRobot;
  } // namespace comau
}  // namespace cnr


#endif