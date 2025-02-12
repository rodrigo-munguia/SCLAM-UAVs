#ifndef COMPOSITION__PLOT_COMPONENT_HPP_
#define COMPOSITION__PLOT_COMPONENT_HPP_

#include "visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/simple_serv.hpp"
#include <opencv2/viz.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <utility>
#include <cinttypes>
#include <string>
#include "interfaces/msg/frame.hpp"
#include "interfaces/msg/lslam.hpp"
#include "interfaces/msg/gmap.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

using namespace cv;
using namespace std;



namespace plot
{

class PLOTscene : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit PLOTscene(const rclcpp::NodeOptions & options);

  ~PLOTscene();

protected:
  void on_timer();  

private:
   
   thread plot_loop_;
   void PLOT_LOOP();
   //static void KeyboardViz3d(const viz::KeyboardEvent &w, void *t);

  //viz::Viz3d viewer("Viewer");
   viz::Viz3d viewer;
   bool viewer_active;

  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  
  // declare subscribers
  rclcpp::Subscription<interfaces::msg::Frame>::SharedPtr sub_Frame_;
  rclcpp::Subscription<interfaces::msg::Lslam>::SharedPtr sub_lslam_data_;
  rclcpp::Subscription<interfaces::msg::Gmap>::SharedPtr sub_gmap_data_;
  // webots ground truth
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_Gps_;
    
  // declare callbacks
  void Frame_callback(const interfaces::msg::Frame & msg) const;
  void lslam_data_callback(const interfaces::msg::Lslam & msg);
  void gmap_data_callback(const interfaces::msg::Gmap & msg);
  // for webots ground truth
  void GT_callback(const geometry_msgs::msg::PointStamped & msg);
  
  //declare Services
  rclcpp::Service<interfaces::srv::SimpleServ>::SharedPtr srv_plot_run_;
  void Handle_plot_run_service(const std::shared_ptr<interfaces::srv::SimpleServ::Request> request,std::shared_ptr<interfaces::srv::SimpleServ::Response> response);
  

  void PLOTscene::draw_drone();
  void PLOTscene::draw_diff_robot();
  void PLOTscene::set_camera_view();
  double Cam_distance;
  bool New_cam_view = false;
  bool New_ss = false;
  int Cam_view = 1;  // 1-> x-y  2-> x-z  3-> y-z 
  double Cam_pos_init_x, Cam_pos_init_y, Cam_pos_init_z;
  double Cam_zoom_delta, Cam_move_delta;
  std::mutex mutex_set_cam;

  //------
  cv::Mat frame;
  std::mutex mutex_frame;
  cv::Affine3d robot_pose;
  double robot_uncertainty;
  cv::Affine3d robot_camera_pose;
  cv::Mat EKFmap;
  cv::Mat EKFmap_color;
  cv::Mat ANCHORSmap;
  cv::Mat ANCHORSmap_color;
  std::mutex mutex_get_state;
  std::mutex mutex_clear_scene;
  std::mutex mutex_clear_get_gmap;
  std::mutex mutex_webots_state;
  bool new_clear_scene;
  bool new_robot_pose_flag;
  std::vector<cv::Point3f> LocalSlam_trajectory;
  std::vector<cv::Point3f> Actual_trajectory;
  std::vector<cv::Point2d> Img_matched_feats;
  std::vector<cv::Point2d> Img_unmatched_feats;
  std::vector<cv::Point2d> Img_matched_anchors;
  std::vector<cv::Point2d> Img_unmatched_anchors;
  std::vector<cv::Affine3d> kf_poses;
  cv::Mat Gmap;
  cv::Mat Gmap_color;

  bool init_scene;
  bool slam_start_flag;
  void draw_localslam_trajectory();

  void reset_scene();
  void initialize_scene();
  void take_screenshot();

  int n_ss;
  
  // parameters
  void setParameters();
  int Viewer_width;
  int Viewer_height;
  bool Draw_local_slam_trajectory;
  
  bool Plot_actual_cam_pos;
  bool Draw_actual_trayectory;
  bool Plot_local_map;
  bool Plot_global_map;

  double Robot_cam_pos_x;
  double Robot_cam_pos_y;
  double Robot_cam_pos_z;
  double Robot_cam_axis_x;
  double Robot_cam_axis_y;
  double Robot_cam_axis_z;
  string Robot_type;

  
};

}  // namespace composition

#endif  // COMPOSITION__LISTENER_COMPONENT_HPP_