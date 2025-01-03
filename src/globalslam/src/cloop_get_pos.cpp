#include "cloop.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"



// convert an Armadillo matrix to OpenCV matrix. NOTE: no copy is made
template <typename T>
cv::Mat_<T> to_cvmat(const arma::Mat<T> &src)
{
  return cv::Mat_<double>{int(src.n_cols), int(src.n_rows), const_cast<T*>(src.memptr())};
}

bool CLOOP::Get_pos_of_current_kf(arma::vec::fixed<3> &pos,KEYFRAME &kf_cl, std::vector<int> &idx_pt_matches, std::vector<cv::Point2d> &image_points, int step)
{

    vector<cv::Point3f> Points3D;
    vector<cv::Point2f> Points2D;

    for (int i = 0; i < idx_pt_matches.size();i++)
    {
        int64 idx_pt = idx_pt_matches[i];

        double x = Gmap.AnchorsDATA[idx_pt].AnchorState[0]; 
        double y = Gmap.AnchorsDATA[idx_pt].AnchorState[1];
        double z = Gmap.AnchorsDATA[idx_pt].AnchorState[2];

        Points3D.push_back(cv::Point3f(x, y, z)); 

        cv::Point2d uv;
        uv = Undistort_a_point(image_points[i],cam_parameters,1);
        //uv = Distort_a_point(image_points[i],cam_parameters,1);

        Points2D.push_back(cv::Point2f(uv.x, uv.y)); 

    }

/*
    void PnPProblem::estimatePoseRANSAC( const std::vector<cv::Point3f> &list_points3d, // list with model 3D coordinates
                                     const std::vector<cv::Point2f> &list_points2d,     // list with scene 2D coordinates
                                     int flags, cv::Mat &inliers, int iterationsCount,  // PnP method; inliers container
                                     float reprojectionError, double confidence )    // Ransac parameters
*/


//double Rn2c[9] = {kf_cl.Rn2c.at(0,0), kf_cl.Rn2c.at(1,0), kf_cl.Rn2c.at(2,1) , kf_cl.Rn2c.at(0,1), kf_cl.Rn2c.at(1,1), kf_cl.Rn2c.at(2,1), kf_cl.Rn2c.at(0,2), kf_cl.Rn2c.at(1,2), kf_cl.Rn2c.at(2,2)    };
//double roll,pitch,yaw;
//Ra2b_to_Euler_colum_major(roll, pitch, yaw, Rn2c);
//double rvec_ini[3] = {roll,pitch,yaw}; // consider ned to xyz convertion
//cv::Mat rvec = cv::Mat(3, 1, CV_64FC1,rvec_ini); 

arma::mat Rn2c_t = kf_cl.Rn2c.t(); // for converting to OpenCV mat    
cv::Mat R = to_cvmat(Rn2c_t); // Kf_1 Projection matrix (OpenCv format)

cv::Mat rvec;
cv::Rodrigues(R, rvec);
//cout << rvec_r << endl;

double tvec_ini[3] = {kf_cl.t_c2n[0],kf_cl.t_c2n[1],kf_cl.t_c2n[2] }; // consider ned to xyz convertion
cv::Mat tvec = cv::Mat(3, 1, CV_64FC1,tvec_ini); 


cv::Mat A_matrix_;
A_matrix_ = cv::Mat::zeros(3, 3, CV_64FC1);   // intrinsic camera parameters
    A_matrix_.at<double>(0, 0) = cam_parameters.fc[0];       //      [ fx   0  cx ]
    A_matrix_.at<double>(1, 1) = cam_parameters.fc[1];       //      [  0  fy  cy ]
    A_matrix_.at<double>(0, 2) = cam_parameters.cc[0];       //      [  0   0   1 ]
    A_matrix_.at<double>(1, 2) = cam_parameters.cc[1];
    A_matrix_.at<double>(2, 2) = 1;

cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
cv::Mat inliers;

bool useExtrinsicGuess = true;   // if true the function uses the provided rvec and tvec values as
    // initial approximations of the rotation and translation vectors

 // RANSAC parameters
    int iterationsCount = 500;      // number of Ransac iterations.
    float reprojectionError = 1.0;  // maximum allowed distance to consider it an inlier.
    double confidence = 0.99;       // ransac successful confidence.

  int pnpMethod = cv::SOLVEPNP_ITERATIVE;
  //int pnpMethod =cv::SOLVEPNP_EPNP; 
      

  
  //cout << "intial att: " << rvec << endl;
  cv::Mat tvec_u = cv::Mat::zeros(3, 1, CV_64FC1);
  tvec_u.at<double>(0) = tvec.at<double>(0);
  tvec_u.at<double>(1) = tvec.at<double>(1);
  tvec_u.at<double>(2) = tvec.at<double>(2);
  cv::Mat rvec_u = cv::Mat::zeros(3, 1, CV_64FC1);
  rvec_u.at<double>(0) = rvec.at<double>(0);
  rvec_u.at<double>(1) = rvec.at<double>(1);
  rvec_u.at<double>(2) = rvec.at<double>(2);

  
  cv::solvePnPRansac( Points3D, Points2D, A_matrix_, distCoeffs, rvec, tvec,
                        useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                        inliers, pnpMethod );
  
 //cv::solvePnP(Points3D, Points2D, A_matrix_, distCoeffs, rvec, tvec,useExtrinsicGuess,pnpMethod);
 // tvec.at<double>(1) = -tvec.at<double>(1);
 // tvec.at<double>(2) = -tvec.at<double>(2);
   // test --------------------

  if(!inliers.empty())
  {  
    double sum_e = 0;    
    double n_in = 0;
    
    for(int i = 0; i < Points3D.size(); i++)
      {   
          if(inliers.at<char>(i) != 0) 
            {
              arma::vec::fixed<3> Pt;
              Pt[0] = Points3D[i].x;
              Pt[1] = Points3D[i].y; 
              Pt[2] = Points3D[i].z;
              arma::vec::fixed<3> Tn2c;
              Tn2c[0] = tvec.at<double>(0);
              Tn2c[1] = tvec.at<double>(1);
              Tn2c[2] = tvec.at<double>(2);
              arma::vec::fixed<3> pc = kf_cl.Rn2c*(Pt - Tn2c); // feature point expressed in the camera frame
              arma::mat::fixed<2,3> duv_dPc;
              cv::Point2d uvd = Projection_model(pc,1,false,cam_parameters,duv_dPc );
              int sm =  0;
              // check if the Anchor is predicted to appear in the image 
              cv::Point2d uvm = Points2D[i];
              /*
              if((uvd.x > sm)&&(uvd.x < PAR.Mono_cam_img_cols - sm)&&(uvd.y > sm)&&(uvd.y < PAR.Mono_cam_img_rows - sm))
              {
   
   
  
                in++;
              }
              */
             
              if(uvd.x == -1 || uvd.y == -1)
              {
                sum_e = 100000000;
                break;
              }
              
              double d = sqrt( pow(uvd.x - uvm.x,2) + pow(uvd.y - uvm.y,2)  );
              sum_e = sum_e + d;
              n_in++;
            }    

      }
      
      double mean_reprojection_error  = sum_e/n_in;
      cout << "Potential Loop: Mean reprojection error: " << mean_reprojection_error  << endl;
      n_cloop_intents++;
      

      double d_xy = sqrt( pow(tvec_u.at<double>(0) - tvec.at<double>(0),2) +  pow(tvec_u.at<double>(1) - tvec.at<double>(1),2) );


      //if (mean_reprojection_error < PAR.CL_min_mean_reprojection_error && d_xy < PAR.CL_xy_update_max_delta)
      if (mean_reprojection_error < PAR.CL_min_mean_reprojection_error && d_xy < kf_cl.radius_uncertainty+PAR.CL_xy_update_max_delta)
      { 
        // The pose computed by the PnP technique es admited.      
        // cout << "Close loop: " << endl;
        // cout << "Initial pose: " << tvec_u.t() << endl;
        // cout << "PnP pose:     " << tvec.t() << endl;
        //cout << "intial rot: " << rvec_u << endl; 
        //cout << "PnP rot: " << rvec << endl;
        pos[0] = tvec.at<double>(0);
        pos[1] = tvec.at<double>(1);
        pos[2] = tvec.at<double>(2);
        
        // store statitics 
        if(PAR.Stats){
          CLOOP_Stat cloop_stat;
          cloop_stat.n_loop_closing_intents = n_cloop_intents;
          cloop_stat.reprojection_error = mean_reprojection_error;
          cloop_stat.error_correction = d_xy;
          cloop_stat.step = step;
          store.cloop_stats.push_back(cloop_stat);
        }  

        return true;           
      }
      else
      {
         return false;
      }
   
  }
  else
  {
    return false;
  }


}
//-----------------------------------------------------------
bool CLOOP::Get_pos_of_current_kf_from_Home_tag(arma::vec::fixed<3> &pos, KEYFRAME &kf_cl)
{

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

        
        cv::aruco::detectMarkers(kf_cl.frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
            
            
        if (!markerIds.empty()) {
             

               std::vector<cv::Vec3d> rvecs, tvecs;
               cv::aruco::estimatePoseSingleMarkers(markerCorners, 1.0, cameraMatrix, distCoeffs, rvecs, tvecs);
               
               std::vector<cv::Point2f> markerCorner = markerCorners[0];
               cv::Point2f center = (markerCorner[0] + markerCorner[2]) / 2.0;
               
               double d_px = sqrt(pow(cx-center.x,2)+pow(cy-center.y,2));
               
               /*
               ROBOT_STATE r_v;
               r_v.x = -tvecs[0][1];
               r_v.y = -tvecs[0][0];
               r_v.z = tvecs[0][2];
               //r_v.z = r_a.z; // use drone altimeter for altitude
               */
              double x_a = tvecs[0][1];
              double y_a = -tvecs[0][0];
              double z_a = tvecs[0][2];

              //cout << "cloop-> Home Tag detected! " << endl;
              //cout << "t: "<< x_a << " " << y_a << " " << z_a <<  endl;
              //cout << "s: "<< kf_cl.r_N[0] << " " << kf_cl.r_N[1] << " " << kf_cl.r_N[2] <<  endl; 
              
              double d =  sqrt(pow(x_a,2) + pow(y_a,2));
              
              if (d < PAR.CL_tag_max_xy_dis && d_px < PAR.CL_tag_max_xy_dis_px)
              {
                pos[0] = x_a;
                pos[1] = y_a;
                pos[2] = z_a;
                return true;
              }
              


        }



return false;
}