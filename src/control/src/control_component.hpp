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

#ifndef COMPOSITION__CONTROL_COMPONENT_HPP_
#define COMPOSITION__CONTROL_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "interfaces/srv/simple_serv.hpp"
#include "interfaces/msg/alt.hpp"
#include "interfaces/msg/att.hpp"
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/range.hpp"
#include "interfaces/msg/gps.hpp"
#include "interfaces/msg/spd.hpp"
#include "interfaces/msg/ododiff.hpp"
#include "interfaces/msg/odovw.hpp"
#include "interfaces/msg/robotstate.hpp"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "parameters.hpp"
#include "../../localslam/src/localslam_types.hpp"
#include "../../common/Vision/vision.hpp"
#include <opencv2/aruco.hpp>
#include <cv_bridge/cv_bridge.h>

#include "control_types.hpp"
#include "logger.hpp"

namespace control
{

  class ControlQuad : public rclcpp::Node
  {
  public:
    COMPOSITION_PUBLIC
    explicit ControlQuad(const rclcpp::NodeOptions & options);

    ~ControlQuad();

  protected:
    //void on_timer();

  private:
    
    parameters PAR;
    
    bool actual_z_flag;
    void reinit_control();

    thread control_loop_;
    void Control_Loop();
    bool control_thread_loop_run;

    

    std::mutex mutex_webots_state;
    std::mutex mutex_slam_state;
    std::mutex mutex_robot_state;
    std::mutex mutex_cl;
    std::mutex mutex_frame;
    cv::Mat frame;
    bool cl_flag;

    ROBOT_STATE r_c;  // robot feedback state
    ROBOT_STATE r_a;  // robot actual state (from webots)
    ROBOT_STATE r_s;  // robot estimated state (from SLAM)
    
    
    // declare subscribers
    
    // webot buscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_Imu_;    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_Gps_;

    rclcpp::Subscription<interfaces::msg::Robotstate>::SharedPtr sub_Slam_;

    rclcpp::Subscription<interfaces::msg::Frame>::SharedPtr sub_Frame_;        

    // declare publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_; 
    
    
    // declare Services      
      rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv_control_run_;
      void Handle_control_run_service(const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,std::shared_ptr<interfaces::srv::SimpleServ::Response> response);
    
    // declare clients
      rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_ekf_run_;
      rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_gslam_run_;
      rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_webot_run_;
      rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_plot_;
    
      

    //------   
    void setParameters();
    void setSubscribers();
    std::mutex mutex_run_plan;
    bool run_control_plan;
    
    // control plan parser
    int cmd_idx; // command (instruction index)
    void load_control_plan();
    
    vector<CMD> Control_plan;
    bool execute_cmd();

    bool go_point(ROBOT_STATE &r_d);
    bool go_point_using_visual_mark(ROBOT_STATE &r_d);
    double compute_ys(double lambda_y);
    bool recognize_home();
    bool explore_area(double x_a,double y_a);
    bool go_home();

    void Imu_callback(const sensor_msgs::msg::Imu & msg);   
    void Gps_callback(const geometry_msgs::msg::PointStamped & msg);
    void Slam_callback(const interfaces::msg::Robotstate & msg);
    void Frame_callback(const interfaces::msg::Frame & msg);

    void SaveScreenShot();
    
    double restrictAngle(double angle);
    
    CAM cam_parameters;
    arma::vec::fixed<3> t_c2r;  // vector defining the position of the camara respect to the robot frame 
    arma::mat::fixed<3,3> Rr2c; // Robot to camera rotation matrix
   
    LoggerGoPoint *logger_gp; // for loggind go-point command data

    LoggerGoPoint *logger_vp; // for loggind go-point command data
  };

}  // namespace composition

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_