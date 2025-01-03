#include "control_component.hpp"
#include "../../common/Vision/vision.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
using namespace std;




void rotationVectorToEulerAngles(cv::Vec3d rotationVector, double& roll, double& pitch, double& yaw)
{
    // Convert rotation vector to rotation matrix
    cv::Mat rotationMatrix;
    cv::Rodrigues(rotationVector, rotationMatrix);

    // Extract Euler angles from rotation matrix
    double r11 = rotationMatrix.at<double>(0, 0);
    double r21 = rotationMatrix.at<double>(1, 0);
    double r31 = rotationMatrix.at<double>(2, 0);
    double r32 = rotationMatrix.at<double>(2, 1);
    double r33 = rotationMatrix.at<double>(2, 2);

    roll = std::atan2(r32, r33);
    pitch = std::atan2(-r31, std::sqrt(r32 * r32 + r33 * r33));
    yaw = std::atan2(r21, r11);
}

namespace control
{
    // 
    bool ControlQuad::go_point_using_visual_mark(ROBOT_STATE &r_d)
    {
        cout << "-> go to point respect visual mark: " << r_d.x << " " << r_d.y << "  " << r_d.z << " " << r_d.yaw   << endl;
        
        bool succes = false;
        bool run = false;

        mutex_run_plan.lock();
           run = run_control_plan;           
        mutex_run_plan.unlock();

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        
        double fx = cam_parameters.fc[0];
        double fy = cam_parameters.fc[1];
        double cx = cam_parameters.cc[0];
        double cy = cam_parameters.cc[1];
        double k1 = cam_parameters.distortions[0];
        double k2 = cam_parameters.distortions[1];
        double p1 = cam_parameters.distortions[2];
        double p2 = cam_parameters.distortions[3];
        double k3 = cam_parameters.distortions[4];

        cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);

        bool x_s = false;
        bool y_s = false;
        bool z_s = false;
        bool yaw_s = false;

        bool mark_detected = false;

        while(run == true && succes == false)
        {   
            cv::Mat img;
            mutex_frame.lock(); 
                img = frame;
            mutex_frame.unlock();

            //cout << img.rows << endl;    
            cv::aruco::detectMarkers(img, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
            
            //cout << markerIds.size() << endl;
            if (!markerIds.empty()) {
               // cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
            if(mark_detected == false)
            {
                cout << "control-> Home tag detected!" << endl;
                mark_detected = true;
            }

               std::vector<cv::Vec3d> rvecs, tvecs;
               cv::aruco::estimatePoseSingleMarkers(markerCorners, 1.0, cameraMatrix, distCoeffs, rvecs, tvecs);
               
               ROBOT_STATE r_v;
               r_v.x = tvecs[0][1];
               r_v.y = tvecs[0][0];
               r_v.z = tvecs[0][2];
               //r_v.z = r_a.z; // use drone altimeter for altitude

               double roll, pitch, yaw;
               rotationVectorToEulerAngles(rvecs[0], roll, pitch, yaw);
               //r_v.yaw = yaw - 1.5707963268;
               r_v.yaw = yaw + 1.5707963268 ;
                           
               
               // compute errors
               double e_x = r_d.x - r_v.x;
               double e_y = r_d.y - r_v.y;
               double e_z = r_d.z - r_v.z;
               double e_yaw = r_d.yaw - r_v.yaw;
               e_yaw = restrictAngle(e_yaw);
               logger_vp->log(r_d.x,r_v.x,r_d.y,r_v.y,r_d.z,r_v.z,r_d.yaw,r_v.yaw);

               //cout << "t: "<< r_v.x << " " << r_v.y << " " << r_v.z << " " << r_v.yaw << " " <<  endl;
               //cout << "a: " << r_a.x << " " << r_a.y << " " << r_a.z << " " << r_a.yaw << " " << endl;
               //cout << "e: " << e_x << " " << e_y << " " << e_z << " " << e_yaw << " " << endl;

               if(abs(e_x) < PAR.cp_x)x_s = true; 
                    else x_s = false ;
               if(abs(e_y) < PAR.cp_y)y_s = true;
                    else y_s = false;
               if(abs(e_z) < PAR.cp_z)z_s = true;
                    else z_s = false;
               if(abs(e_yaw) < PAR.cp_yaw)yaw_s = true; 
                    else yaw_s = false;

               // if all the errors have been minimized
                if((x_s == true && y_s == true && z_s == true && yaw_s == true) )
                {   
                    succes = true;
                    auto twist_msg = geometry_msgs::msg::Twist();
                    twist_msg.linear.x = 0;    // Linear velocity along x-axis
                    twist_msg.linear.y = 0;    // Linear velocity along y-axis
                    twist_msg.linear.z = 0;    // Linear velocity along y-axis                         
                    twist_msg.angular.z = 0;
                    pub_cmd_vel_->publish(twist_msg);
                    return true;
                }
                //--------------------------------------------------------
                // --- compute x-y control -------------------------------            
                //double a = yaw;
                double a = e_yaw;                           
                arma::mat::fixed<2,2> Rn2b =  {{cos(a), sin(a)},
                                            { -sin(a), cos(a)}};
                arma::vec::fixed<2> e_xy_N;
                e_xy_N[0] = e_x;
                e_xy_N[1] = e_y;
                
                arma::vec::fixed<2> e_xy_B;
                // get x-y error in body frame
                e_xy_B = Rn2b*e_xy_N;        
                    //cout << "e_xy_B x: " << e_xy_B[0] << " e_xy_B y: " << e_xy_B[1] << endl;                       
                
                // Apply logarithmic smoothing to the position error
                double kp_x = PAR.kp_x;
                double kp_y = PAR.kp_y;
                double es_x_B = kp_x*log10(fabs(e_xy_B[0]) + 1.0) * (e_xy_B[0] >= 0 ? 1.0 : -1.0);
                double es_y_B = kp_y*log10(fabs(e_xy_B[1]) + 1.0) * (e_xy_B[1] >= 0 ? 1.0 : -1.0);           
                    //cout << es_x_B << " " << es_y_B <<  endl;
                // Calculate the control signal based on the smoothed error
                double controlSignal_x = es_x_B ;
                double controlSignal_y = es_y_B ;
                // Apply control signal limits if necessary
                double maxControlSignal_xy = PAR.maxControlSignal_xy;  // Maximum allowed control signal
                double minControlSignal_xy = PAR.minControlSignal_xy; // Minimum allowed control signal
                controlSignal_x = std::max(std::min(controlSignal_x, maxControlSignal_xy), minControlSignal_xy);
                controlSignal_y = std::max(std::min(controlSignal_y, maxControlSignal_xy), minControlSignal_xy);
                            
                //--------------------------------------------------------
                // --- compute z control -------------------------------  
                double e_z_N = e_z;
                double kp_z = PAR.kp_z;
                double es_z_N = kp_z*log10(fabs(e_z_N) + 1.0) * (e_z_N >= 0 ? 1.0 : -1.0);
                double controlSignal_z = es_z_N ;
                double maxControlSignal_z = PAR.maxControlSignal_z;  // Maximum allowed control signal
                double minControlSignal_z = PAR.minControlSignal_z; // Minimum allowed control signal
                controlSignal_z = std::max(std::min(controlSignal_z, maxControlSignal_z), minControlSignal_z);
                
                //--------------------------------------------------------
                // --- compute yaw control -------------------------------     
                
                double kp_yaw = PAR.kp_yaw;
                double es_yaw = kp_yaw*log10(fabs(e_yaw) + 1.0) * (e_yaw >= 0 ? 1.0 : -1.0);
                double controlSignal_yaw = es_yaw;
                double maxControlSignal_yaw = PAR.maxControlSignal_yaw;  // Maximum allowed control signal
                double minControlSignal_yaw = PAR.minControlSignal_yaw; // Minimum allowed control signal
                controlSignal_yaw = std::max(std::min(controlSignal_yaw, maxControlSignal_yaw), minControlSignal_yaw);

                //----- sent vel coomands to robot---6-------------------------
                auto twist_msg = geometry_msgs::msg::Twist();
                twist_msg.linear.x = controlSignal_x;    // Linear velocity along x-axis
                twist_msg.linear.y = controlSignal_y;    // Linear velocity along y-axis
                twist_msg.linear.z = controlSignal_z;    // Linear velocity along y-axis                         
                twist_msg.angular.z = controlSignal_yaw;

                //cout << "u: " << twist_msg.linear.x << " " << twist_msg.linear.y << " " << twist_msg.linear.z << " " << twist_msg.angular.z << " " << endl;               
                pub_cmd_vel_->publish(twist_msg);            
                // Delta_t = .02 sec  (50 Hz) 
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 

                mutex_run_plan.lock();
                    run = run_control_plan;
                mutex_run_plan.unlock();
            }
            else
            {   
                // visual mark is not detected
                mark_detected = false;
                cout << "control-> The home tag is not being detected.." << endl;
                cout << "control-> go up" << endl;               
                // go up for trying to find home tag
               
                
                    auto twist_msg = geometry_msgs::msg::Twist();
                    twist_msg.linear.x = 0;    // Linear velocity along x-axis
                    twist_msg.linear.y = 0;    // Linear velocity along y-axis
                    twist_msg.linear.z = PAR.maxControlSignal_z;    // Linear velocity along y-axis                         
                    twist_msg.angular.z = 0;
                    pub_cmd_vel_->publish(twist_msg);
                    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // 
                    
                  double z = r_a.z; // use drone altimeter for altitude
                             
                
                if (z > 6)
                {
                    cout << "control-> The Home tag has not been detected.." << endl;
                    cout << "control-> stopping control plan.." << endl;
                    
                    auto twist_msg = geometry_msgs::msg::Twist();
                        twist_msg.linear.x = 0;    // Linear velocity along x-axis
                        twist_msg.linear.y = 0;    // Linear velocity along y-axis
                        twist_msg.linear.z = 0;    // Linear velocity along y-axis                         
                        twist_msg.angular.z = 0;
                        pub_cmd_vel_->publish(twist_msg);

                    return true;
                }
            }
        }
    }


    
    //-------------------------------------------------------------------------
    bool ControlQuad::go_point(ROBOT_STATE &r_d)
    {
        cout << "-> go to point (a): " << r_d.x << " " << r_d.y << "  " << r_d.z << " " << r_d.yaw   << endl;

        bool succes = false;
        bool run = false;

        mutex_run_plan.lock();
           run = run_control_plan;           
        mutex_run_plan.unlock();
               
        bool x_s = false;
        bool y_s = false;
        bool z_s = false;
        bool yaw_s = false;

        while(run == true && succes == false )
        {            
            // error = target_pos - current_pos  (in navigation frame)
            mutex_robot_state.lock();
                double yaw = r_c.yaw;
                double e_x = r_d.x - r_c.x;
                double e_y = r_d.y - r_c.y;
                double e_z = r_d.z - r_c.z;
                double e_yaw = r_d.yaw - yaw;
                logger_gp->log(r_d.x,r_c.x,r_d.y,r_c.y,r_d.z,r_c.z,r_d.yaw,yaw);
            mutex_robot_state.unlock();
            
            e_yaw = restrictAngle(e_yaw);

            if(abs(e_x) < PAR.cp_x)x_s = true; 
                else x_s = false ;
            if(abs(e_y) < PAR.cp_y)y_s = true;
                else y_s = false;
            if(abs(e_z) < PAR.cp_z)z_s = true;
                else z_s = false;
            if(abs(e_yaw) < PAR.cp_yaw)yaw_s = true; 
                else yaw_s = false;           
            
            bool cl_loop_event = false;
            mutex_cl.lock();
             cl_loop_event = cl_flag;
            mutex_cl.unlock();

            // if a close loop event is detected, stop, and return with true; 
            if((x_s == true && y_s == true && z_s == true && yaw_s == true) || cl_loop_event == true)
            {   
                auto twist_msg = geometry_msgs::msg::Twist();
                twist_msg.linear.x = 0;    // Linear velocity along x-axis
                twist_msg.linear.y = 0;    // Linear velocity along y-axis
                twist_msg.linear.z = 0;    // Linear velocity along y-axis                         
                twist_msg.angular.z = 0;
                pub_cmd_vel_->publish(twist_msg);
                return true;
            }
            
            //--------------------------------------------------------
            // --- compute x-y control -------------------------------            
            double a = yaw;                            
            arma::mat::fixed<2,2> Rn2b =  {{cos(a), sin(a)},
                                        { -sin(a), cos(a)}};
            arma::vec::fixed<2> e_xy_N;
            e_xy_N[0] = e_x;
            e_xy_N[1] = e_y;
            
            arma::vec::fixed<2> e_xy_B;
            // get x-y error in body frame
            e_xy_B = Rn2b*e_xy_N;        
                //cout << "e_xy_B x: " << e_xy_B[0] << " e_xy_B y: " << e_xy_B[1] << endl;                       
            
            // Apply logarithmic smoothing to the position error
            double kp_x = PAR.kp_x;
            double kp_y = PAR.kp_y;
            double es_x_B = kp_x*log10(fabs(e_xy_B[0]) + 1.0) * (e_xy_B[0] >= 0 ? 1.0 : -1.0);
            double es_y_B = kp_y*log10(fabs(e_xy_B[1]) + 1.0) * (e_xy_B[1] >= 0 ? 1.0 : -1.0);           
                //cout << es_x_B << " " << es_y_B <<  endl;
            // Calculate the control signal based on the smoothed error
            double controlSignal_x = es_x_B ;
            double controlSignal_y = es_y_B ;
            // Apply control signal limits if necessary
            double maxControlSignal_xy = PAR.maxControlSignal_xy;  // Maximum allowed control signal
            double minControlSignal_xy = PAR.minControlSignal_xy; // Minimum allowed control signal
            controlSignal_x = std::max(std::min(controlSignal_x, maxControlSignal_xy), minControlSignal_xy);
            controlSignal_y = std::max(std::min(controlSignal_y, maxControlSignal_xy), minControlSignal_xy);
                         
            //--------------------------------------------------------
            // --- compute z control -------------------------------  
            double e_z_N = e_z;
            double kp_z = PAR.kp_z;
            double es_z_N = kp_z*log10(fabs(e_z_N) + 1.0) * (e_z_N >= 0 ? 1.0 : -1.0);
            double controlSignal_z = es_z_N ;
            double maxControlSignal_z = PAR.maxControlSignal_z;  // Maximum allowed control signal
            double minControlSignal_z = PAR.minControlSignal_z; // Minimum allowed control signal
            controlSignal_z = std::max(std::min(controlSignal_z, maxControlSignal_z), minControlSignal_z);
            
            //--------------------------------------------------------
            // --- compute yaw control -------------------------------     
            
            double kp_yaw = PAR.kp_yaw;
            double es_yaw = kp_yaw*log10(fabs(e_yaw) + 1.0) * (e_yaw >= 0 ? 1.0 : -1.0);
            double controlSignal_yaw = es_yaw;
            double maxControlSignal_yaw = PAR.maxControlSignal_yaw;  // Maximum allowed control signal
            double minControlSignal_yaw = PAR.minControlSignal_yaw; // Minimum allowed control signal
            controlSignal_yaw = std::max(std::min(controlSignal_yaw, maxControlSignal_yaw), minControlSignal_yaw);

            //----- sent vel coomands to robot----------------------------
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = controlSignal_x;    // Linear velocity along x-axis
            twist_msg.linear.y = -controlSignal_y;    // Linear velocity along y-axis
            twist_msg.linear.z = controlSignal_z;    // Linear velocity along y-axis                         
            twist_msg.angular.z = controlSignal_yaw;

            //cout << "x-y-z: " << r_c.x << " " << r_c.y << " " << r_c.z << " " << r_c.yaw << " u: " << controlSignal_x << " " << controlSignal_y << " " << controlSignal_z << " " << controlSignal_yaw << endl;
            //cout << "des  : " << r_d.x << " " << r_d.y << "  " << r_d.z << " " << r_d.yaw   << endl;
            //cout << "error: " << e_x << " " << e_y << "  " << e_z << " " << e_yaw   << endl;

            //cout << "actual: " << r_a.x << " " << r_a.y << "  " << r_a.z << " " << r_a.yaw   << endl;
            //cout << "slam  : " << r_s.x << " " << r_s.y << "  " << r_s.z << " " << r_s.yaw   << endl;
            
            pub_cmd_vel_->publish(twist_msg);            
            // Delta_t = .02 sec  (50 Hz) 
            std::this_thread::sleep_for(std::chrono::milliseconds(50)); // 

            mutex_run_plan.lock();
                run = run_control_plan;
            mutex_run_plan.unlock();
        }   


        cout << "Control -> Something went wrong with go_point cmd.." << endl;
        return false;
    }
    
    // compute superposition lenght ys (m), given superposition factor lambda_y
    // ys depends on the altitude and field of view of the camera.
    double ControlQuad::compute_ys(double lambda_y)
    {
         arma::mat::fixed<3,3> Rc2r = Rr2c.t();
         double height = r_c.z;
         double phi = r_c.roll;
         double theta = r_c.pitch;
         double psi = r_c.yaw;
         double Ra2b[9];
         Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
         arma::mat Rn2r(Ra2b,3,3); //  navigation to robot rotation matrix         
         arma::mat::fixed<3,3> Rr2n = Rn2r.t();
         arma::mat Rc2n = Rr2n*Rc2r;
         cv::Point2d uvd;
         // define a "virtual" feature located along the ray defined by the camera center and the image corner
         uvd.x = PAR.Mono_cam_img_cols;
         uvd.y = PAR.Mono_cam_img_rows;
         arma::vec::fixed<3> hc;
         arma::mat::fixed<3,2> dhc_duvd;
         hc = Inverse_projection_model(uvd,1,false, cam_parameters,dhc_duvd); // compute inverse projection model 
         //arma::vec::fixed<3> hn = Rc2n*hc;   // 
         arma::vec::fixed<3> hn = hc;   // 
         arma::vec::fixed<3> m = hn/arma::norm(hn); // normalized vector (in the nav frame) pointing in the direction of the feature
         arma::vec::fixed<3> v = {0,0,1};
         arma::vec::fixed<3> vn =  Rc2n*v;
         // compute the aproximate depth of the "virtual" feature
         //double depth = (abs(height)) /arma::dot(m,vn); 
         double depth = (abs(height)) /arma::dot(m,v); 
         // compute the x-y-z position of the virtual feature vf_N expressed in the Camera frame
         arma::vec::fixed<3> pv_C =  depth*m;    //  d_v = Rr2n*Rc2r*P_c
         
        double yv = abs(pv_C(1));        
        double ys = yv + yv*lambda_y;   
       
       return ys;
    }



    bool ControlQuad::recognize_home()
    {
        mutex_robot_state.lock();
            ROBOT_STATE r0 = r_c;
        mutex_robot_state.unlock();

        double lambda_y = PAR.Rec_Home_lambda_y;   // ----------
        double ys = compute_ys(lambda_y);         
        double y_a = ys; 
        double x_a = ys; 

        

        bool succes = explore_area(x_a,y_a);
        if(succes==false)return false;

        succes = go_point(r0);
        if(succes == false)return false;

        cout << "Control -> Exploring home area done " << endl;

        return true;
    }

    bool ControlQuad::explore_area(double x_a,double y_a)
    {   
        cout << "Control -> Exploring home area from: (x_C: " << x_a << " y_C: " << y_a << ")   to (x_C: " << -x_a  << " y_C: " << -y_a << ")"<< endl; 
        SaveScreenShot(); 

        double lambda_y = PAR.Explore_area_lambda_y;   // ----------      
        double ys = compute_ys(lambda_y);  
        
        // compute number of segments to "divide" the area to explore
        int n_seg_y = int(y_a/ys);
        if (n_seg_y == 0) n_seg_y = 1;
        // compute lenght(m) of segmentes
        double dy = y_a/(double)n_seg_y;
        double py = y_a;

        vector<ROBOT_STATE> wp; // for storing way-points
        //vector<double> x_i,y_i;
        
        // get current robot state        RecHome
        mutex_robot_state.lock();
            ROBOT_STATE r0 = r_c;
        mutex_robot_state.unlock();
        
        // intial way-point (robot state)
        ROBOT_STATE p;
        p.x = x_a + r0.x;
        p.y = -py + r0.y;
        p.z = r0.z;
        p.yaw = r0.yaw;
        //p.y = -p.y; // invert the y-order of exploration
        wp.push_back(p); 

        bool right = 0;  
        int c_lr = 0;

        for(int i = 1; i < (n_seg_y*4 +2)  ; i++)
        {      
            ROBOT_STATE p;      
            if(right == 1)
            {
                p.x = x_a + r0.x;
                p.y = -py + r0.y;
                c_lr ++;
                    if(c_lr == 2) 
                    {
                        right = 0;
                        c_lr = 0;
                    }
                    else
                    {
                        py = py - dy;
                    }     
            }
            else
            {
                p.x = -x_a + r0.x; 
                p.y = -py + r0.y;
                
                c_lr ++;
                if(c_lr == 2) 
                {
                    right = 1;
                    c_lr = 0;
                }
                else
                {
                    py = py - dy;
                } 
            }       
            p.z = r0.z;
            p.yaw = r0.yaw;
            
            //p.y = -p.y; // invert the y-order of exploration
            wp.push_back(p);         
            
            
        }

        for(int i = 0; i<wp.size(); i++)
        {
            cout << "Control-> go to x: " << wp[i].x << " y: " << wp[i].y << endl;
            bool succes = go_point(wp[i]);
            if(succes == false)return false;

            bool cl_loop_event = false;
            mutex_cl.lock();
             cl_loop_event = cl_flag;
            mutex_cl.unlock();
            if (cl_loop_event == true)
            {
                // a CL has been detected! probably during go home command
                return true;
            }
            
        }

        SaveScreenShot();

    return true;
    }
    

    //---------------------------------------------------------------------------
    bool ControlQuad::go_home()
    {
        //cout << "Control -> Going to home.. " << endl;
        SaveScreenShot();
        
        mutex_robot_state.lock();
            ROBOT_STATE r0 = r_c;
        mutex_robot_state.unlock();

        ROBOT_STATE p;
        p.x = 0;
        p.y = 0;
        p.z = r_c.z;
        p.yaw = 0;
        bool succes = go_point(p);  

        if ( succes == true)
        {    
            bool cl_loop_event = false;
            mutex_cl.lock();
                cl_loop_event = cl_flag;
            mutex_cl.unlock();
            if (cl_loop_event == true)
            {
               cl_flag = false;
               cout << "control -> Close Loop detected going home .." << endl;
               SaveScreenShot();
               ROBOT_STATE p;
               p.x = 0;
               p.y = 0;
               p.z = r_c.z;;
               p.yaw = 0;
               bool succes = go_point(p);      
               //cout << "Control -> Going to home.. " << endl;
               if(succes == true)
               {
                  cout << "control -> Going to home Succes!!! " << endl; 
               }            
            }
            else
            {
                // No Close Loop has been detected when going home..
                // try to explore the uncertanty area.
                 cout << "control -> No CL detected when going home.. " << endl;
                 cout << "control -> Exploring area.. " << endl;
                 mutex_robot_state.lock();
                    ROBOT_STATE r = r_c; // get current robot state
                 mutex_robot_state.unlock();

                 double r_u = r.r_u; // get robot uncertainty 
                 double y_a = r_u; 
                 double x_a = r_u; 
                 bool succes = explore_area(x_a,y_a);
                 
                    bool cl_loop_event = false;
                    mutex_cl.lock();
                        cl_loop_event = cl_flag;
                    mutex_cl.unlock();
                    if (cl_loop_event == true)
                    {
                        cl_flag = false;
                        cout << "control -> Close Loop detected during exploration .." << endl;
                        SaveScreenShot();
                        ROBOT_STATE p;
                        p.x = 0;
                        p.y = 0;
                        p.z = r_c.z;;
                        p.yaw = 0;
                        bool succes = go_point(p);      
                        //cout << "Control -> Going to home.. " << endl;
                        if(succes == true)
                        {
                            cout << "control -> Going to home Succes!!! " << endl;
                            SaveScreenShot(); 
                        }            
                    }
                    else
                    {
                        cout << "control-> No CL has been detected during exploration...." << endl;
                    } 

            }       

        }
        




     return true;   
    }




}    