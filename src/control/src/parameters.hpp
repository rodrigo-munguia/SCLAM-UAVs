/*---------------------------------------------------
Rodrigo Munguía 2021.

System parameters
-----------------------------------------------------
*/
#include <string>

#ifndef PARAMETERS_C_H
#define PARAMETERS_C_H

using namespace std;



struct parameters
{   
    
    
    string Control_plan_path;
    
    bool Slam_feedback;
    // proportional gains for control
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_yaw;
    // Maximum/minimun allowed control signals
    double maxControlSignal_xy;  
    double minControlSignal_xy;  
    double maxControlSignal_z;
    double minControlSignal_z;
    double maxControlSignal_yaw;
    double minControlSignal_yaw;
    // control precision
    double cp_x;
    double cp_y;
    double cp_z;
    double cp_yaw;

    // Monocular camera parameters    
    int Mono_cam_img_rows;
    int Mono_cam_img_cols;
    std::vector<double> Mono_cam_distortions;
    double Mono_cam_cc_u;
    double Mono_cam_cc_v;
    double Mono_cam_fc_u;
    double Mono_cam_fc_v;
    double Mono_cam_alpha_c;
    // camera to robot position/orientation
    double Mono_cam_2_robot_axis_x;
    double Mono_cam_2_robot_axis_y;
    double Mono_cam_2_robot_axis_z;
    double Mono_cam_2_robot_pos_x;
    double Mono_cam_2_robot_pos_y;
    double Mono_cam_2_robot_pos_z;
    // Overlapping parameters for exploration
    double Explore_area_lambda_y;
    double Rec_Home_lambda_y;
        


} ;   

#endif /*PARAMETERS_H*/



