// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef COMPOSITION__WEBOTb_COMPONENT_HPP_
#define COMPOSITION__WEBOTb_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "../../../install/interfaces/include/interfaces/srv/simple_serv.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "interfaces/msg/odovw.hpp"
#include "interfaces/msg/rangetobase.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "parameters.hpp"

//#include <cv_bridge/cv_bridge.h>
#include "../../localslam/src/localslam_types.hpp"


namespace webotbridge
{

  class WEBOTb : public rclcpp::Node
  {
  public:
    COMPOSITION_PUBLIC
    explicit WEBOTb(const rclcpp::NodeOptions & options);

    ~WEBOTb();

  protected:
    //void on_timer();

  private:
    
    parameters PAR;
    void loop();
    void setParameters();
    double addGaussianNoise(double value, double stdDev);
    
    // declare subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_Imu_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_Camera_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_Range_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_Gps_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_GpsSpeed_;

    // declare publishers
    rclcpp::Publisher<interfaces::msg::Alt>::SharedPtr pub_Alt_;
    rclcpp::Publisher<interfaces::msg::Att>::SharedPtr pub_Att_;
    rclcpp::Publisher<interfaces::msg::Gps>::SharedPtr pub_Gps_;
    rclcpp::Publisher<interfaces::msg::Range>::SharedPtr pub_Range_;
    rclcpp::Publisher<interfaces::msg::Frame>::SharedPtr pub_Frame_;
    rclcpp::Publisher<interfaces::msg::Spd>::SharedPtr pub_Spd_;       
    rclcpp::Publisher<interfaces::msg::Ododiff>::SharedPtr pub_OdoD_;
    rclcpp::Publisher<interfaces::msg::Odovw>::SharedPtr pub_OdoV_;
    rclcpp::Publisher<interfaces::msg::Rangetobase>::SharedPtr pub_RangeToBase_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_GT_;
    
    

   
    
    // declare Services
      //rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv1_;   
    // declare clients
      //rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_ekf_run_;
    
    // For ros2 message to opencv MAT convertion 
    sensor_msgs::msg::Image::SharedPtr img_msg;

    //------
    rclcpp::TimerBase::SharedPtr timer_;

    
    void Imu_callback(const sensor_msgs::msg::Imu & msg) ;
    void Camera_callback(const sensor_msgs::msg::Image & msg);
    void Range_callback(const sensor_msgs::msg::Range & msg);
    void Gps_callback(const geometry_msgs::msg::PointStamped & msg);
    void GpsSpeed_callback(const geometry_msgs::msg::Vector3 & msg);
      //void setParameters();

    bool Range_available; // flag
    double Range; // data   
    
    double restrictAngle(double angle);
    arma::mat Rn2rG;
    double init_yaw;  
   

  };

}  // namespace composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_