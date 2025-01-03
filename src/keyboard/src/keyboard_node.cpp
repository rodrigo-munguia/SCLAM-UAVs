#include <cstdio>
#include "iostream"
#include "string"
#include "vpKeyboard.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;


///---------------------------------
 void printcommands()
 {
    std::cout << "\n| Dataset commands :\n"             
                   "|   'q' to quit.\n"
                   "|   's'-> Start/Stop SLAM          w-> Reset Control-SLAM\n"
                   "|   'a'-> Start/Stop Control Plan                \n"
                   "|   '-'-> zoom out '+' ->  zoom in '1'-> x-y view '2' -> x-z view  '3'->y-z view\n"
                   "|   '8'-> view up '5' ->  view down '4'-> view left '6' -> view right  '7'-> clear plot\n"
                   "|   '9'-> save screenshot '0' -> log statistics\n"
                   "|   't'-> TakeOff / landing \n"   
                   "|         r (forward)                  i (up)         \n"
                   "| d (left)          g(right)   j(turn L)     l (turn R) \n"
                   "|         f (back)                     k(down)           \n"

                 << std::endl;
   
 }   


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  vpKeyboard keyboard;
  int k = 0;

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("keyboard_dataset");
  
  // create client services

  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_dataset =
    node->create_client<interfaces::srv::SimpleServ>("dataset_service");

  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_ekf_run_ =
    node->create_client<interfaces::srv::SimpleServ>("ekf_run_service");
  
  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_globalslam_run_ =
    node->create_client<interfaces::srv::SimpleServ>("globalslam_run_service");

  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_control_run_ =
    node->create_client<interfaces::srv::SimpleServ>("control_run_service");

  rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_webot_service_ =
    node->create_client<interfaces::srv::SimpleServ>("webot_service");  
  // --------------

  // create publishers ----------------------------

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
  
  //-------------------------------------

/*

 auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
           request->cmd = 1;

 while (!client_dataset->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  } 
          

  auto result = client_dataset->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  { 
    if (result.get()->response == true)
    {
      cout << "keyboard->  Dataset component response: Ok" << endl;
    }
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %ld", result.get()->response);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "keyboard->  Failed to call service dataset_service");
  }

//----------------------------------------------------------------
//  check for local slam component
rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_localslam =
    node->create_client<interfaces::srv::SimpleServ>("ekf_run_service");

auto request_2 = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request_2->cmd = 'r';           
auto result_2 = client_localslam->async_send_request(request_2);

if (rclcpp::spin_until_future_complete(node, result_2) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                  { 
                    if (result_2.get()->response == true) cout << "keyboard->  Local slam component response: Ok" << endl;
                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %ld", result.get()->response);
                  } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "keyboard->  Failed to call service ekf_run_service");
                  } 
//-----------------------------------------------------------------
//  check for plot component
*/
rclcpp::Client<interfaces::srv::SimpleServ>::SharedPtr client_plot =
    node->create_client<interfaces::srv::SimpleServ>("plot_service");

/*
auto request_3 = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request_3->cmd = 'r';           
auto result_3 = client_plot->async_send_request(request_3);

if (rclcpp::spin_until_future_complete(node, result_3) ==
                    rclcpp::FutureReturnCode::SUCCESS)
                  { 
                    if (result_3.get()->response == true) cout << "keyboard->  Plot component response: Ok" << endl;
                    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %ld", result.get()->response);
                  } else {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "keyboard->  Failed to call plot_service");
                  } 
*/
bool to = false;

printcommands();
bool start_slam_flag = false;

while (k != 'q') 
      {
 
         k = '0'; // If no key is hit, we send a non-assigned key
         if (keyboard.kbhit()) 
         {
           k = keyboard.getchar();
          
           
           if (k == 's' )
           {  
              if(start_slam_flag == false)
              { 
                start_slam_flag = true;
                // Start/Stop LocalSLAM;              
                auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
                request->cmd = 'r';              
                auto result = client_ekf_run_->async_send_request(request);
                request->cmd = 'u';              
                result = client_ekf_run_->async_send_request(request); 
              }
              else
              {
                auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
                request->cmd = 's';              
                auto result = client_ekf_run_->async_send_request(request);
              }

            }
            if (k == 'a' )
            { 
              // Start/Stop Control plan
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = 'r';
              auto result = client_control_run_->async_send_request(request);
              
            }
            if (k == 'w' )
            { 
                // Restart LocalSLAM, Gmap, Cloop, Control;              
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = 'i';
              auto result = client_ekf_run_->async_send_request(request); 
              result = client_globalslam_run_->async_send_request(request);
              result = client_control_run_->async_send_request(request);
              request->cmd = 'c';
              result = client_plot->async_send_request(request); 

            }
            
            if ( k == '0')
            {
              // log statistics
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_ekf_run_->async_send_request(request); 
              result = client_globalslam_run_->async_send_request(request); 

            }
            if ( k == 45)
            {
              // '-' ->  plot zoom out
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            if ( k == 43)
            {
              // '+' ->  plot zoom in
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request);
            }
            if ( k == '1')
            {
              // '1' ->  x-y view 
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request);
            }
            if ( k == '2')
            {
              // '2' ->  x-z view
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            if ( k == '3')
            {
              // '3' ->  y-z view 
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            if ( k == '8')
            {
              // '8' ->  cam up 
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            if ( k == '4')
            {
              // '4' ->  cam left 
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            if ( k == '6')
            {
              // '6' ->  cam right 
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            if ( k == '5')
            {
              // '5' ->  cam down 
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            if ( k == '7')
            {
              // 'c' ->  clear plot
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            if (k == '9')
            {
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              request->cmd = k;
              auto result = client_plot->async_send_request(request); 
            }
            
            //cout << k << endl;
            if( k == ' ')
            {
              // Stop
              auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.linear.x = 0.0;    // Linear velocity along x-axis
              twist_msg.linear.y = 0.0;
              twist_msg.linear.z = 0.0;
              twist_msg.angular.z = 0.0;      
              pub_cmd_vel_->publish(twist_msg);             
              cout << "stop" << endl;             
              
            } 
            if( k == 'i')
            {
              // Up
              //auto twist_msg = std::make_shared<geometry_msgs::msg::Twist>();
              auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.linear.z = 0.25;    // Linear velocity along x-axis
              pub_cmd_vel_->publish(twist_msg);
              
            } 
        
            if( k == 'k')
            {
              // Down              
               auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.linear.z = -0.25;    // Linear velocity along x-axis
              pub_cmd_vel_->publish(twist_msg);          
              
            }
        
            if( k == 'l')
            {
              // turn Right
              auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.angular.z =  -0.5;    // Linear velocity along x-axis
              pub_cmd_vel_->publish(twist_msg);  
              
            }
        
            if( k == 'j')
            {
              // turn Left
              auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.angular.z =  0.5;    // Linear velocity along x-axis
              pub_cmd_vel_->publish(twist_msg);  
              
            }
        
            if( k == 'r')
            {
              // Forward
              auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.linear.x =  0.5;    // Linear velocity along x-axis
              pub_cmd_vel_->publish(twist_msg);  
              
            }
        
            if( k == 'f')
            {
              // Backward
              auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.linear.x =  -0.5;    // Linear velocity along x-axis
              pub_cmd_vel_->publish(twist_msg);  
              
            }
        
            if( k == 'd')
            {
              //  left
              auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.linear.y =  0.5;    // Linear velocity along x-axis
              pub_cmd_vel_->publish(twist_msg);  
              
            }
        
            if( k == 'g')
            {
              //  right
              auto twist_msg = geometry_msgs::msg::Twist();
              twist_msg.linear.y =  -0.5;    // Linear velocity along x-axis
              pub_cmd_vel_->publish(twist_msg);  
              
            }
            if (k == 't')
            {
              auto request = std::make_shared<interfaces::srv::SimpleServ::Request>();
              if(to == false)
              {
                request->cmd = 't';
                to = true;
              }
              else{
                request->cmd = 'l';
                to = false;
              }
              auto result = client_webot_service_->async_send_request(request);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10)); // sleep a short period of time to save proccesor use
            
            


            printcommands();
            

         }
         //running = handleKeyboardInput(drone, k,ekf,locks,par,gmap,cloop,control,stop_control);
         

         
       }

  return 0;
}
