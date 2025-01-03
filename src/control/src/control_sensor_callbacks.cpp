#include "control_component.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
using namespace std;


namespace control
{
  
  double ControlQuad::restrictAngle(double angle) {
    // Normalize the angle to the range -2π to +2π
    angle = fmod(angle, 2 * M_PI);

    // Adjust the angle to the range -π to +π
    if (angle <= -M_PI)
        angle += 2 * M_PI;
    else if (angle > M_PI)
        angle -= 2 * M_PI;

    return angle;
}


  void ControlQuad::Imu_callback(const sensor_msgs::msg::Imu & msg) 
  { 
    static double init_yaw;   
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
    
    if(init_yaw_flag== false)
    {
        init_yaw = yaw;
        init_yaw_flag = true;
    }
    else
    {   
      mutex_webots_state.lock();  
        r_a.yaw = restrictAngle(yaw-init_yaw);
        r_a.roll = roll;
        r_a.pitch = pitch;
      mutex_webots_state.unlock(); 

        if(PAR.Slam_feedback == false)
        { 
          mutex_robot_state.lock();          
            r_c.yaw = restrictAngle(yaw-init_yaw);
          mutex_robot_state.unlock();           
        }
       
    }
  
  }
    
  //-----------------------------------------------------------------------
  void ControlQuad::Gps_callback(const geometry_msgs::msg::PointStamped & msg)
  { 
    

    double x = msg.point.x;
    double y = msg.point.y;
    double z = msg.point.z;
    
    mutex_webots_state.lock();
      r_a.x = x;
      r_a.y = y;
      r_a.z = z;
      actual_z_flag = true;
    mutex_webots_state.unlock();
    
    if(PAR.Slam_feedback == false)
        {
            mutex_robot_state.lock();
                r_c.x = x;
                r_c.y = y;
                r_c.z = z;
            mutex_robot_state.unlock();
        }

  }

  //-----------------------------------------------------------------------
  void ControlQuad::Slam_callback(const interfaces::msg::Robotstate & msg)
  {
    static double init_yaw;   
    static bool init_yaw_flag = false;
    static double init_z;

    double x = msg.robot_state[7];
    double y = msg.robot_state[8];
    double z = msg.robot_state[9];
    double roll = msg.robot_state[1];
    double pitch = msg.robot_state[2];
    double yaw = msg.robot_state[3];

    double r_u = msg.radius_uncertainty;
    
    if(init_yaw_flag== false)
    {
        init_yaw = yaw;
        init_yaw_flag = true;
        init_z = r_a.z;
    }
    else
    {  
      mutex_slam_state.lock();
        r_s.x = x;
        r_s.y = y;
        r_s.z = z;
        r_s.yaw = restrictAngle(yaw-init_yaw);
        r_s.roll = roll;
        r_s.pitch = pitch;
        r_s.r_u = r_u;
      mutex_slam_state.unlock();  


      if(PAR.Slam_feedback == true && actual_z_flag == true)
      {
          mutex_robot_state.lock();
                  r_c.x = x;
                  r_c.y = y;
                  r_c.z = -z + init_z;
                  r_c.roll = roll;
                  r_c.pitch = -pitch;
                  r_c.yaw = -restrictAngle(yaw-init_yaw);
                  r_c.r_u = r_u;
          mutex_robot_state.unlock();
      }
    }  


  }  
  //----------------------------------------------------
  void ControlQuad::Frame_callback(const interfaces::msg::Frame & msg) 
{
  
  
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg.img,sensor_msgs::image_encodings::MONO8 );
  
   
  mutex_frame.lock(); 
    frame = cv_ptr->image;
  mutex_frame.unlock();    

  


}




}    