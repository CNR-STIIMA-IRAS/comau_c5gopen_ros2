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

 
#include <boost/shared_ptr.hpp>

namespace boost {
#ifdef BOOST_NO_EXCEPTIONS
void throw_exception( std::exception const & e ) { throw 11; };
#endif
}


#include <comau_c5gopen_ros2/json.hpp>

#include <comau_c5gopen_ros2/mqtt_driver.hpp>

namespace cnr
{
  namespace comau
  {
    // void tic(int mode) 
    // {
    //   static std::chrono::high_resolution_clock::time_point t_start;
    
    //   if (mode==0)
    //       t_start = std::chrono::high_resolution_clock::now();
    //   else 
    //   {
    //     auto t_end = std::chrono::high_resolution_clock::now();
    //     ROS_WARN_STREAM_THROTTLE(1.0, "Elapsed time is " << (t_end-t_start).time()*1E-9 << "  seconds" );
    //   }
    // }

    // void toc() 
    // { 
    //   tic(1); 
    // }

    void ComauMsgDecoder::on_message(const struct mosquitto_message *msg)
    {
      char* buffer = new char[msg->payloadlen];
      memcpy(buffer, msg->payload, msg->payloadlen);

      nlohmann::json data = nlohmann::json::parse(buffer);
    
      if (mtx_.try_lock_for(std::chrono::milliseconds(1))) //try to lock mutex for 1ms
      {

        for (size_t id=0; id<MSG_AXES_LENGTH; id++)
        {
          std::string joint_str = "J" + std::to_string(id+1);
          mqtt_msg_[std::string(msg->topic)].joint_values_[id] = data[joint_str];
        }

        mqtt_msg_[std::string(msg->topic)].time_ = data["time"];

        mtx_.unlock();

        if (!first_message_rec_)
          first_message_rec_ = true;
      }
      else
      {
        RCLCPP_WARN_STREAM(logger_, "Can't lock mutex in ComauMsgDecoder::on_message timeout reached, last MQTT message not recovered." );
      }

      delete buffer;
    }

    bool ComauMsgDecoder::getLastReceivedMessage(cnr::comau::comau_msg& last_msg, const std::string& topic_name)
    {
      if (mqtt_msg_.find(topic_name) == mqtt_msg_.end()) 
      {
        RCLCPP_WARN_STREAM(logger_, "getLastReceivedMessage: can't find the topic data: " << topic_name );
        return false; 
      }
      else
      { 
        last_msg = mqtt_msg_[topic_name];
        return true;
      }
    }      
    
    void ComauMsgEncoder::on_publish(int mid)
    {
      // Nothing to do here
    }

    MQTTComauClient::MQTTComauClient(const char *id, const char *host, const int port, int keepalive)
    {
      try
      {
        mqtt_msg_enc_ = new cnr::comau::comau_msg;
        mqtt_msg_dec_ = new cnr::comau::comau_msg;
      
        comau_msg_encoder_ = new cnr::comau::ComauMsgEncoder();
        comau_msg_decoder_ = new cnr::comau::ComauMsgDecoder(logger_);

        mqtt_client_ = new cnr::mqtt::MQTTClient(id, host, port, comau_msg_encoder_, comau_msg_decoder_);
      }
      catch(const std::exception& e)
      {
        RCLCPP_ERROR_STREAM(logger_, "Exception thrown in MQTTComauClient constructor: " <<  e.what() );
      }
    }

    MQTTComauClient::~MQTTComauClient()
    {  
      delete mqtt_msg_dec_;
      delete mqtt_msg_enc_;
      delete comau_msg_decoder_;
      delete comau_msg_encoder_;
      delete mqtt_client_;
    }

    int MQTTComauClient::stop()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->stop();      

      return -1;
    }

    int MQTTComauClient::loop(int timeout)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->loop(timeout);
      
      return -1;
    }

    int MQTTComauClient::reconnect()
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->reconnect();
      
      return -1;
    }

    int MQTTComauClient::subscribe(int *mid, const char *sub, int qos)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->subscribe(mid, sub, qos);
      
      return -1;
    }
    
    int MQTTComauClient::unsubscribe(int *mid, const char *sub)
    {
      if (mqtt_client_ != NULL)
        return mqtt_client_->unsubscribe(mid, sub);
      
      return -1;
    }

    int MQTTComauClient::publish(const void* payload, int& payload_len, const char* topic_name)
    {        
      if (mqtt_client_ != NULL)
        return mqtt_client_->publish(payload, payload_len, topic_name);
      
      return -1;
    }

    bool MQTTComauClient::getLastReceivedMessage(cnr::comau::comau_msg& last_msg, const std::string& topic_name )
    {
      if (comau_msg_decoder_ != NULL)
      {
        if (!comau_msg_decoder_->isFirstMsgRec())
        {
          RCLCPP_WARN_STREAM_THROTTLE(logger_, clock_, 2000, "First message not received yet." );
          return false;
        }

        if (comau_msg_decoder_->mtx_.try_lock_for(std::chrono::milliseconds(1))) //try to lock mutex for 1ms
        {
          if ( !comau_msg_decoder_->getLastReceivedMessage(last_msg, topic_name))
          {
            comau_msg_decoder_->mtx_.unlock();
            return false;
          }  

          comau_msg_decoder_->mtx_.unlock();
          return true;
        }
        else
        {
          RCLCPP_WARN_STREAM(logger_, "Can't lock mutex MQTTComauClient::getLastReceivedMessage. Last message received from MQTT not recovered." );
          return false;
        }          
      
      }

      return false;
    }
  }
}