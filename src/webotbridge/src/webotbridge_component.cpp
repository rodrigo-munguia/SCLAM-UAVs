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

#include "webotbridge_component.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <random>
#include <cmath>
#include "../../common/Transforms/Euler_to_Ra2b.hpp" 



using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

namespace webotbridge
{

// Create a Talker "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.

  

  WEBOTb::WEBOTb(const rclcpp::NodeOptions & options)
  : Node("webotb", options)
  {
    setParameters();
    
    // Set subscribers
    sub_Imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&WEBOTb::Imu_callback, this, _1)); 
    sub_Camera_ = this->create_subscription<sensor_msgs::msg::Image>("camera", 10, std::bind(&WEBOTb::Camera_callback, this, _1)); 
    sub_Range_ = this->create_subscription<sensor_msgs::msg::Range>("ds0", 10, std::bind(&WEBOTb::Range_callback, this, _1)); 
    sub_Gps_ = this->create_subscription<geometry_msgs::msg::PointStamped>("webot_robot/gps", 10, std::bind(&WEBOTb::Gps_callback, this, _1)); 
    sub_GpsSpeed_ = this->create_subscription<geometry_msgs::msg::Vector3>("webot_robot/gps/speed_vector", 10, std::bind(&WEBOTb::GpsSpeed_callback, this, _1)); 

    // Create a publisher of "std_mgs/String" messages on the "chatter" topic.
    //pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);
    
    pub_Alt_ = create_publisher<interfaces::msg::Alt>("Altitude_topic",10);
    pub_Att_ = create_publisher<interfaces::msg::Att>("Attitude_topic",10);
    pub_Gps_ = create_publisher<interfaces::msg::Gps>("Gps_topic",10);
    pub_Range_ = create_publisher<interfaces::msg::Range>("Range_topic",10);
    pub_Frame_ = create_publisher<interfaces::msg::Frame>("Frame_topic",10); 
    pub_Spd_ = create_publisher<interfaces::msg::Spd>("Speed_topic",10);
    pub_OdoD_ = create_publisher<interfaces::msg::Ododiff>("OdometryD_topic",10);
    pub_OdoV_ = create_publisher<interfaces::msg::Odovw>("OdometryV_topic",10);      
    pub_RangeToBase_ = create_publisher<interfaces::msg::Rangetobase>("Rangetobase_topic",10);  
    pub_GT_ = create_publisher<geometry_msgs::msg::PointStamped>("GT_topic",10);  

     

    Range_available = false;
    //loop();
    Rn2rG.eye(); 
    
  }

  WEBOTb::~WEBOTb()
  { 
    
  }
   
  //---------------------------------------------------
  // Set paramters 
  void WEBOTb::setParameters()
  {  
    //  Declare node parameters (and default values)
    this->declare_parameter<double>("Sigma_roll", 0.017453);
    this->declare_parameter<double>("Sigma_pitch", 0.017453);
    this->declare_parameter<double>("Sigma_yaw", 0.032);
    this->declare_parameter<double>("Sigma_Alt", 0.1);    
    this->declare_parameter("Base_location", std::vector<double>{0.0, 0.0, 0.0});  
    this->declare_parameter<double>("Sigma_rangetobase", 0.25); 
    this->declare_parameter<double>("Sigma_Spd", 0.05);   

    
    
    
    // Set parameter struct
    this->get_parameter("Sigma_roll",PAR.Sigma_roll);
    this->get_parameter("Sigma_pitch",PAR.Sigma_pitch);
    this->get_parameter("Sigma_yaw",PAR.Sigma_yaw); 
    this->get_parameter("Sigma_Alt",PAR.Sigma_Alt); 
    rclcpp::Parameter Base_location_par = this->get_parameter("Base_location");
    PAR.Base_location = Base_location_par.as_double_array(); 
    this->get_parameter("Sigma_rangetobase",PAR.Sigma_rangetobase);
    this->get_parameter("Sigma_Spd",PAR.Sigma_Spd);      

  }
  //-----------------------------------------------------------

  void WEBOTb::loop()
  {

      while(true)
      {
        
         std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep a short period of time to save proccesor use
      }


  }

  // Function to add Gaussian noise to a variable
  double WEBOTb::addGaussianNoise(double value, double stdDev)
  {
      std::random_device rd;
      std::mt19937 generator(rd());
      std::normal_distribution<double> distribution(0.0, stdDev);

      double noise = distribution(generator);
      return value + noise;
  } 
  //----------------------------------------------------------------------------------
  
  void WEBOTb::Imu_callback(const sensor_msgs::msg::Imu & msg) 
  {     
     static bool init_yaw_flag = false;

    //cout << msg.orientation.x << " " << msg.orientation.y << " "<<  msg.orientation.z << " " << msg.orientation.w << endl;
    tf2::Quaternion quat;
    quat.setX(msg.orientation.x);
    quat.setY(msg.orientation.y);
    quat.setZ(msg.orientation.z);
    quat.setW(msg.orientation.w);
    //tf2::fromMsg(msg.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    //RCLCPP_INFO(rclcpp::get_logger("example_node"), "Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);
    auto message = interfaces::msg::Att();
    message.roll = addGaussianNoise(roll, PAR.Sigma_roll);
    message.pitch = addGaussianNoise(-pitch, PAR.Sigma_pitch);
    message.yaw = addGaussianNoise(-yaw+0.00, PAR.Sigma_yaw);
    //message.time = dat.att.time;        
    pub_Att_->publish(message);

    


    if(init_yaw_flag == false)
    {
      init_yaw = yaw;
      init_yaw_flag = true;
    }
    else
    {
      // Store rotation matrix for later use
      double Ra2b[9];      
      //yaw = restrictAngle(yaw-init_yaw);
      yaw = restrictAngle(yaw);
      Euler_to_Ra2b_colum_major(roll, pitch, yaw, Ra2b);
      arma::mat Rn2r(Ra2b,3,3);
      Rn2rG = Rn2r;
    }

  
  }
  
  //------------------------------------------------------------------------
  void WEBOTb::Camera_callback(const sensor_msgs::msg::Image & msg) 
  { 

    auto message = interfaces::msg::Frame();
    if(Range_available == true)
    {
      message.range = Range;
      Range_available = false;
    }
    else
    {
      message.range = -1;
      
    }

    //cout << message.range << endl;

    message.img = msg;
    //message.time = dat.frame.time; 
    pub_Frame_->publish(message);

  }
  //------------------------------------------------------------------------
  void WEBOTb::Range_callback(const sensor_msgs::msg::Range & msg) 
  { 

    //cout << msg.range << endl;
    Range = msg.range;
    Range_available = true;
    
    auto message = interfaces::msg::Range();
    message.range = msg.range;

    pub_Range_->publish(message);

  }    
  //-----------------------------------------------------------------------
  void WEBOTb::Gps_callback(const geometry_msgs::msg::PointStamped & msg)
  {
    double x = msg.point.x;
    double y = msg.point.y;
    double z = msg.point.z;
    //cout << "GPS:     " << x << " " << y << " " << z << endl;

    arma::vec gt_N = {x, y, z };
    arma::vec gt_R = Rn2rG*gt_N;
    
    auto message_g = geometry_msgs::msg::PointStamped();
    message_g.point.x = gt_R[0];
    message_g.point.y  = -gt_R[1];
    message_g.point.z  = -z;
    
    pub_GT_->publish(message_g);

    

    
    // Emulate altitude measurements
    auto message = interfaces::msg::Alt();
    message.altitude = addGaussianNoise(z, PAR.Sigma_Alt);
    pub_Alt_->publish(message);
    
    
    // Emulate range to base measurements
    //cout << "BASE location:     " << PAR.Base_location[0] << " " << PAR.Base_location[1] << " " << PAR.Base_location[2] << endl;
    // 3D range
    //double range = std::sqrt(std::pow(x - PAR.Base_location[0], 2) + std::pow(y - PAR.Base_location[1], 2) + std::pow(z - PAR.Base_location[2], 2));
    // 2D range
    double range = std::sqrt(std::pow(x - PAR.Base_location[0], 2) + std::pow(y - PAR.Base_location[1], 2) );
    //cout << "RANGE:     " << range << endl;
    auto message_r = interfaces::msg::Rangetobase();
    message_r.range = addGaussianNoise(range, PAR.Sigma_rangetobase);
    pub_RangeToBase_->publish(message_r);
    

  }

  void WEBOTb::GpsSpeed_callback(const geometry_msgs::msg::Vector3 & msg)
  {
    arma::vec sp_N = {msg.x, msg.y, msg.z };
     
     //cout << "GPS speed N:     " << msg.x << " " << msg.y << " " << msg.z << endl;
    
    arma::vec sp_R = Rn2rG*sp_N;
     
     //cout << "GPS speed R:     " << sp_R[0] << " " << sp_R[1]  << " " << sp_R[2]  << endl;
   
    auto message = interfaces::msg::Spd();
    message.speed_x = addGaussianNoise(sp_R[0],PAR.Sigma_Spd);
    message.speed_y = addGaussianNoise(-sp_R[1],PAR.Sigma_Spd);
    message.speed_z = addGaussianNoise(-sp_R[2],PAR.Sigma_Spd);
   
    pub_Spd_->publish(message);
  }



  double WEBOTb::restrictAngle(double angle) {
    // Normalize the angle to the range -2π to +2π
    angle = fmod(angle, 2 * M_PI);

    // Adjust the angle to the range -π to +π
    if (angle <= -M_PI)
        angle += 2 * M_PI;
    else if (angle > M_PI)
        angle -= 2 * M_PI;

    return angle;
}



}// namespace webotbridge


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(webotbridge::WEBOTb)
