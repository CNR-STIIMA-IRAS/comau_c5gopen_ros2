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

#include <algorithm>
#include <charconv>
#include <chrono>
#include <limits>
#include <math.h>
#include <set>

#include <rclcpp/duration.hpp>
#include <rcutils/logging_macros.h>
#include <hardware_interface/component_parser.hpp>

#include <comau_c5gopen_ros2/json.hpp>
#include <comau_c5gopen_ros2/comau_c5gopen_hw.hpp>
#include <comau_c5gopen_ros2/logger_color_macros.hpp>


namespace cnr
{
  namespace comau
  {   

    JointComms::JointComms() : Node("comau_c5gopen_hw")
    {
      cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/command_joint_pos",10);
      fb_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/feedback_joint_pos",10);
    }


    double parse_double(const std::string & text)
    {
      double result_value;
      const auto parse_result = std::from_chars(text.data(), text.data() + text.size(), result_value);
      if (parse_result.ec == std::errc())
      {
          return result_value;
      }

      return 0.0;
    }

    CallbackReturn ComauC5GopenHw::on_init(const hardware_interface::HardwareInfo & info)
    {     
      RCLCPP_INFO(logger_, "Initializing comau_c5gopen_hw");
      if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
      {
        return CallbackReturn::ERROR;
      }
      
      std::string ro = info_.hardware_parameters["read_only"];
      RCLCPP_INFO_STREAM(logger_,"\n RO::" << ro);
      read_only_ = ( ro =="true") ? true : false;

      if(read_only_)
          RCLCPP_INFO_STREAM(logger_,"Read only mode active. The robot can be moved from this hardware interface." );

      RCLCPP_INFO_STREAM(logger_,"Using C5Gopen to control the robot!" );


      // Load configuration parameters

      std::string mqtt_client_id = info_.hardware_parameters["mqtt_client_id"];
      RCLCPP_INFO_STREAM( logger_,"MQTT client ID: " << mqtt_client_id );

      std::string mqtt_broker_ip = info_.hardware_parameters["mqtt_broker_ip"];
      RCLCPP_INFO_STREAM( logger_,"MQTT broker IP: " << mqtt_broker_ip );

      std::string mqtt_port_str = info_.hardware_parameters["mqtt_port"];
      RCLCPP_INFO_STREAM( logger_,"MQTT port: " << mqtt_port_str );
      int mqtt_port = std::stoi(mqtt_port_str);
      
      std::string mqtt_loop_timeout_str = info_.hardware_parameters["mqtt_loop_timeout"];
      RCLCPP_INFO_STREAM( logger_,"MQTT allowed timeout " << mqtt_loop_timeout_str );
      mqtt_loop_timeout_ = std::stoi(mqtt_loop_timeout_str);


      RCLCPP_INFO_STREAM( logger_,"Creating MQTT client..." );
      mqtt_client_.reset( new cnr::comau::MQTTComauClient(mqtt_client_id.c_str(), mqtt_broker_ip.c_str(), mqtt_port, mqtt_loop_timeout_) );
      RCLCPP_INFO_STREAM( logger_,"MQTT client created!" );


      topic_fdb_pos_name_ = info_.hardware_parameters["topic_fdb_pos_name"];
      
      if (mqtt_client_->subscribe(NULL, topic_fdb_pos_name_.c_str(), 1) !=0 )
      {
        RCLCPP_ERROR_STREAM( logger_, "Can't subscribe topic: " << topic_fdb_pos_name_.c_str() << " from MQTT broker");
        return CallbackReturn::ERROR;
      }
      else
        RCLCPP_INFO_STREAM( logger_, "Topic: " << topic_fdb_pos_name_.c_str() << " subscribed");


      topic_fdb_vel_name_ = info_.hardware_parameters["topic_fdb_vel_name"];
      
      if (mqtt_client_->subscribe(NULL, topic_fdb_vel_name_.c_str(), 1) !=0 )
      {
        RCLCPP_ERROR_STREAM( logger_, "Can't subscribe topic: " << topic_fdb_vel_name_.c_str() << " from MQTT broker");
        return CallbackReturn::ERROR;
      }
      else
        RCLCPP_INFO_STREAM( logger_, "Topic: " << topic_fdb_vel_name_.c_str() << " subscribed");
      

      if ( info_.joints.size() > MSG_AXES_LENGTH )
      {
        RCLCPP_ERROR_STREAM( logger_, "The number of robot joint of the comau hardware interface is not the one expected the MQTT client. Please remember that the MQTT interface expect 10 joints." );
        return CallbackReturn::ERROR;
      }
      
      joint_positions_.resize(info_.joints.size(), 0);
      joint_velocities_.resize(info_.joints.size(), 0);
      joint_position_command_.resize(info_.joints.size(), 0); 

      for (size_t j=0; j<joint_positions_.size(); j++)
      {
        joint_names_.push_back(info_.joints[j].name);
        RCLCPP_DEBUG_STREAM(logger_, info_.joints[j].name);
      }


      // Read current robot state

      int received_msg_ = false;
      rclcpp::Clock clock = rclcpp::Clock();
      rclcpp::Time start = clock.now();

      while(clock.now()-start <= rclcpp::Duration(10,0))
      {
        cnr::comau::comau_msg last_pos_msg;  
        if( mqtt_client_->getLastReceivedMessage(last_pos_msg, topic_fdb_pos_name_) )
        {
          RCLCPP_INFO_STREAM(logger_, "The current robot position is: " );
          for(size_t idx=0; idx<joint_positions_.size(); idx++)
          {
            joint_positions_.at(idx) = last_pos_msg.joint_values_[idx] * (M_PI/180.0); 
            RCLCPP_INFO_STREAM(logger_, joint_positions_.at(idx));
          }
          received_msg_ = true;
          break;
        }  
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_WARN_STREAM(logger_, "Waiting to receive the first position message from MQTT" );
      }

      if (!received_msg_)
      {
        RCLCPP_ERROR_STREAM( logger_, "After 10 secs can't recover a position message from MQTT, breaking the hardware interface." );
        return CallbackReturn::ERROR;
      }


      received_msg_ = false;
      start = clock.now();

      while(clock.now()-start <= rclcpp::Duration(10,0))
      {
        cnr::comau::comau_msg last_vel_msg;  
        if( mqtt_client_->getLastReceivedMessage(last_vel_msg, topic_fdb_vel_name_) )
        {
          RCLCPP_INFO_STREAM(logger_, "The current robot velocity is: " );
          for(size_t idx=0; idx<joint_velocities_.size(); idx++)
          {
            joint_velocities_.at(idx) = last_vel_msg.joint_values_[idx] * (M_PI/180.0); 
            RCLCPP_INFO_STREAM(logger_, joint_velocities_.at(idx));
          }
          received_msg_ = true;
          break;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        RCLCPP_WARN_STREAM(logger_, "Waiting to receive the first velocity message from MQTT" );
      }


      if (!received_msg_)
      {
        RCLCPP_ERROR_STREAM( logger_, "After 10 secs can't recover a velocity message from MQTT, breaking the hardware interface." );
        return CallbackReturn::ERROR;
      }

      joint_position_command_ = joint_positions_;         
      

      // Write the command with the joint current joint position
      topic_cmd_name_ = info_.hardware_parameters["topic_cmd_name"];

      RCLCPP_INFO_STREAM( logger_, "Publishing position commands on topic: " << topic_cmd_name_.c_str() );
      
      nlohmann::json data;

        
      for (size_t idx=0; idx<joint_position_command_.size(); idx++)
      {
        std::string joint_str = "J" + std::to_string(idx+1);
        data[joint_str.c_str()] = joint_position_command_[idx] * (180.0/M_PI);
      }

    
      if (!read_only_)
      {
        const std::string json_file = data.dump();

        int payload_len = json_file.length() + 1;
        char* payload = new char[ payload_len ];
        strcpy(payload, json_file.c_str());

        if (mqtt_client_->publish(payload, payload_len, topic_cmd_name_.c_str()) != MOSQ_ERR_SUCCESS )
        {
          RCLCPP_ERROR_STREAM( logger_, "Error while publishing the topic " << topic_cmd_name_ );  
          delete payload;
          return CallbackReturn::ERROR;
        }

        delete payload;
      }    

      comms_ = std::make_shared<JointComms>();
      executor_.add_node(comms_);
      std::thread([this]() { executor_.spin(); }).detach();
       
      RCLCPP_INFO_STREAM( logger_, cnr_logger::BOLDGREEN() << "Hardware iterface comau_c5gopen_hw initialized!" << cnr_logger::RESET() ); 

      return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> ComauC5GopenHw::export_state_interfaces()
    {
      std::vector<hardware_interface::StateInterface> state_interfaces;
      for (size_t idx=0; idx<joint_positions_.size(); idx++)
      {
        state_interfaces.emplace_back(info_.joints[idx].name, hardware_interface::HW_IF_POSITION, &joint_positions_[idx]);
        state_interfaces.emplace_back(info_.joints[idx].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[idx]);
      }

      return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ComauC5GopenHw::export_command_interfaces()
    {
      std::vector<hardware_interface::CommandInterface> command_interfaces;

      for (size_t idx=0; idx<joint_position_command_.size(); idx++)
      {
        command_interfaces.emplace_back(info_.joints[idx].name, hardware_interface::HW_IF_POSITION, &joint_position_command_[idx]);
      }

      return command_interfaces;
    }

    return_type ComauC5GopenHw::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
      
      cnr::comau::comau_msg last_pos_msg;  
      if( mqtt_client_->getLastReceivedMessage(last_pos_msg, topic_fdb_pos_name_) )
      {
        for(size_t idx=0; idx<joint_positions_.size(); idx++)
          joint_positions_.at(idx) = last_pos_msg.joint_values_[idx] * (M_PI/180); 
      }  
      else
      {
        RCLCPP_ERROR_STREAM( logger_, "Can't recover the last position message received from MQTT." );
        return return_type::ERROR;
      }
  
      cnr::comau::comau_msg last_vel_msg;  
      if( mqtt_client_->getLastReceivedMessage(last_vel_msg, topic_fdb_vel_name_) )
      {
        for(size_t idx=0; idx<joint_velocities_.size(); idx++)
          joint_velocities_.at(idx) = last_vel_msg.joint_values_[idx] * (M_PI/180); 
      }  
      else
      {
        RCLCPP_ERROR_STREAM( logger_, "Can't recover the last velocity message received from MQTT." );
        return return_type::ERROR;
      }
      
      
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = comms_->get_clock()->now();
      msg.name = joint_names_;
      msg.position = joint_positions_;
      msg.velocity = joint_velocities_;
      comms_->fb_pub_->publish(msg);
      

      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      RCLCPP_DEBUG_STREAM(logger_,"READ time:  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[microseconds]" );

      return return_type::OK;
    }

    return_type ComauC5GopenHw::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
      std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
      
      if(!read_only_)
      {
        nlohmann::json data;        
        for (size_t idx=0; idx<joint_position_command_.size(); idx++)
        {
          std::string joint_str = "J" + std::to_string(idx+1);
          data[joint_str.c_str()] = joint_position_command_[idx] * (180/M_PI);
        }

        // COMAU CONTROLLER WANTS 10 AXES
        // TO BE IMPROVED 
        data["J7"] = 0.0;
        data["J8"] = 0.0;
        data["J9"] = 0.0;
        data["J10"] = 0.0;
            
        const std::string json_file = data.dump();

        int payload_len = json_file.length() + 1;
        char* payload = new char[ payload_len ];
        strcpy(payload, json_file.c_str());

        if (mqtt_client_->publish(payload, payload_len, topic_cmd_name_.c_str()) != MOSQ_ERR_SUCCESS )
        {
          RCLCPP_ERROR_STREAM( logger_, "Error while publishing the topic " << topic_cmd_name_ );  
          delete payload;
          return return_type::ERROR;
        }

        delete payload;
      }
      
      auto msg = sensor_msgs::msg::JointState();
      msg.header.stamp = comms_->get_clock()->now();
      msg.name = joint_names_;
      msg.position = joint_position_command_;
      comms_->cmd_pub_->publish(msg);
      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
      RCLCPP_DEBUG_STREAM(logger_,"WRITE time:  = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[microseconds]" );

      return return_type::OK;
    }
  }

}  // namespace mock_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cnr::comau::ComauC5GopenHw, hardware_interface::SystemInterface)