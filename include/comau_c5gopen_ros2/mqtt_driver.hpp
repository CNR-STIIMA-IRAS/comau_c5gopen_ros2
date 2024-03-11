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


#ifndef __COMAU_MQTT_CLIENT__
#define __COMAU_MQTT_CLIENT__

#include <mutex>
#include <ctime>
#include <chrono>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <cnr_mqtt_client/cnr_mqtt_client.h>

#define MSG_AXES_LENGTH 10 // The length is given by 10 axes robot
#define DEFAULT_KEEP_ALIVE 60


namespace cnr
{
  namespace comau
  {

    // void tic(int mode=0);
    // void toc();

    enum thread_status
    {
      BEFORE_RUN  = 0,
      RUNNING     = 1,
      CLOSED      = 2 
    }__attribute__ ( ( packed ) );

    struct comau_msg 
    {
      double joint_values_[MSG_AXES_LENGTH] = {0};   
      unsigned long long int time_ = 0;
    }; 


    class ComauMsgDecoder: public cnr::mqtt::MsgDecoder
    {
    public:
      ComauMsgDecoder(const rclcpp::Logger& logger): first_message_rec_(false), logger_(logger) {};
      
      // The method should be reimplemented on the base of the application
      void on_message(const struct mosquitto_message *msg) override;
      bool isFirstMsgRec(){return first_message_rec_;};
      bool getLastReceivedMessage(cnr::comau::comau_msg& last_msg, const std::string& topic_name);      

    private:
      rclcpp::Logger logger_;
      std::map<std::string, cnr::comau::comau_msg> mqtt_msg_;
      bool first_message_rec_;
    };

    class ComauMsgEncoder: public cnr::mqtt::MsgEncoder
    {
    public:
      ComauMsgEncoder(){};
      
      // The method should be reimplemented on the base of the application
      void on_publish(int mid) override;

    private:
      // nothing to do here

    };

    class MQTTComauClient
    {
    public:
      MQTTComauClient (const char *id, const char *host, const int port, const int loop_timeout, int keepalive = 60);
      ~MQTTComauClient();

      int stop();
      int loop(const int timeout=4);
      int reconnect();  
      int subscribe(int *mid, const char *sub, int qos);
      int unsubscribe(int *mid, const char *sub);
      int publish(const void* payload, int& payload_len, const char* topic_name);
     
      bool isFirstMsgRec(){ return comau_msg_decoder_->isFirstMsgRec(); };
      bool getLastReceivedMessage(cnr::comau::comau_msg& last_msg, const std::string& topic_name);  
    
    private:
      rclcpp::Clock clock_ = rclcpp::Clock();
      rclcpp::Logger logger_ = rclcpp::get_logger("mqtt_driver");
      cnr::comau::ComauMsgDecoder* comau_msg_decoder_;
      cnr::comau::ComauMsgEncoder* comau_msg_encoder_;

      cnr::comau::comau_msg* mqtt_msg_enc_;
      cnr::comau::comau_msg* mqtt_msg_dec_;            

      cnr::mqtt::MQTTClient* mqtt_client_;

      int loop_timeout_;

      std::thread mqtt_thread_;
      thread_status mqtt_thread_status_ = thread_status::BEFORE_RUN;

      void MQTTThread();
    };
  }

}
#endif