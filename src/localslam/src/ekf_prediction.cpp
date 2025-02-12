#include "ekf.hpp"
//#include "Jac_noise_driven_model.cpp"
#include "ekf_Jacobians.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"


//void Jac_quat_noise_driven_model2(const double x[13], double delta_t, const double k[2],double JFx[169], double JFu[78]);    
//void Jac_euler_noise_driven_model(arma::vec& x, double delta_t, double Tau_r, double Tau_p, arma::mat::fixed<13,13>& Jfx, arma::mat::fixed<13,6>& Jfu);
  

    
  typedef std::vector<double> vec_t;      

void EKF::prediction(double delta_t)
{
  /*
    if(PAR.Prediction_model == "noise_driven")
    {
        //prediction_noise_driven(delta_t);
        prediction_euler_noise_driven(delta_t);
    }
    else if(PAR.Prediction_model == "differential_robot")
    {

        prediction_euler_differential_robot(delta_t);

    }
 */

} 

//--------------------------------------------------------------------------------------------
/*
% Initial States ***************************************************
% x meaning
% index  0    1   2    3   4   5   6    7  8  9  10  11  12  
%       null phi theta psi w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(0) not used
% x(1:3)=   [phi theta psi] -> roll, pitch, and yaw of the robot
% x(4:6)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(7:9)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(10:12)= [v_x v_y v_z]  -> Velocity in body coordinate frame.
*/

void EKF::prediction_euler_odometry_vw_robot(ODOV &odov)
{
    double delta_t = PAR.Robot_odo_delta_t;
   
    double sigma_w = PAR.Sigma_w;
    double sigma_a = PAR.Sigma_a;
    double Tau_r = PAR.Tau_r;
    double Tau_p = PAR.Tau_p;
    double kr = 1/Tau_r;
    double kp = 1/Tau_p; 

    // Attitude Equations
    double phi = x[1];
    double theta = x[2];
    double psi = x[3];
    // body rotational velocities to euler velocities Rotation matrix    
   
   arma::mat::fixed<3,3> R_b2e = {{1 ,  sin(phi)*tan(theta) ,  cos(phi)*tan(theta) },
                                  {0 ,       cos(phi)       ,    -sin(phi)         },
                                  {0 ,  sin(phi)/cos(theta) ,  cos(phi)/cos(theta) }};

   x.subvec(1,3) =  x.subvec(1,3) +  (R_b2e*x.subvec(4,6))*delta_t; // euler_dot = R_b2e*[w_x w_y w_z]';  

   //x.subvec(4,6) = x.subvec(4,6) +  (-kr*x.subvec(4,6))*delta_t;
   x[4] = 0;
   x[5] = 0;
   x[6] = -odov.angular_vel;
   
   
   // Position Equations
   double Ra2b[9];
   Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
   arma::mat Rn2r(Ra2b,3,3); // navigation to robot rotation matrix
   arma::mat::fixed<3,3> Rr2n = Rn2r.t();

   //arma::vec::fixed<3> w_t = {0, 0 , -odov.angular_vel };
   //x.subvec(4,6) = Rn2r*w_t;

 


   x.subvec(7,9) = x.subvec(7,9) + Rr2n*x.subvec(10,12)*delta_t;

   //x.subvec(10,12) = x.subvec(10,12) +  (-kr*x.subvec(10,12))*delta_t;
   
   x[10] = odov.linear_vel;
   x[11] = 0;
   x[12] = 0;

   //------------------------------------
   // Update system covariance matrix P

   arma::mat::fixed<13,13> Jfx;
   arma::mat::fixed<13,6> Jfu;
   
   //cout << x << endl;
   Jac_euler_noise_driven_model(x, delta_t, Tau_r, Tau_p, Jfx, Jfu);

    arma::mat U;
    U.zeros(6,6);
    U(arma::span(0,2),arma::span(0,2)) = arma::eye(3,3)*pow(sigma_w*delta_t,2);
    U(arma::span(3,5),arma::span(3,5)) = arma::eye(3,3)*pow(sigma_a*delta_t,2);

    //cout << Jfx << endl;
    //    cout << Jfu << endl;
    //    cout << P << endl;


    int x_len = x.size();
    if(x_len > PAR.Robot_state_size)
    {   
      //  arma::mat tp = P(arma::span(0,12),arma::span(13,x_len));
      //  arma::mat tj = Jfx*P(arma::span(0,12),arma::span(13,x_len));
        P(arma::span(0,12),arma::span(13,x_len-1)) = Jfx*P(arma::span(0,12),arma::span(13,x_len-1)); 
        P(arma::span(13,x_len-1),arma::span(0,12)) = P(arma::span(0,12),arma::span(13,x_len-1)).t();
        P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + U;
    }
    else
    {
        
        P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + U;
        int q = 10;
    }

    



}
//-------------------------------------------------------------
void EKF::prediction_euler_differential_robot(ODOD &odod)
{   
    double delta_t = PAR.Robot_diff_Delta_t;
    double r = PAR.Robot_diff_wheels_radius;
    double L = PAR.Robot_diff_wheels_distance;
    double w2t_f = PAR.Robot_diff_angular_vel_To_tics_factor;
    

    double w_l = odod.TicksLeft/w2t_f;
    double w_r = odod.TicksRight/w2t_f;

    double v_x = (r/2)*(w_r + w_l);
    double w_z = (r/L)*(w_l - w_r);

   
    double sigma_w = PAR.Sigma_w;
    double sigma_a = PAR.Sigma_a;
    double Tau_r = PAR.Tau_r;
    double Tau_p = PAR.Tau_p;
    double kr = 1/Tau_r;
    double kp = 1/Tau_p; 

    // Attitude Equations
    double phi = x[1];
    double theta = x[2];
    double psi = x[3];
    // body rotational velocities to euler velocities Rotation matrix    
   
   arma::mat::fixed<3,3> R_b2e = {{1 ,  sin(phi)*tan(theta) ,  cos(phi)*tan(theta) },
                                  {0 ,       cos(phi)       ,    -sin(phi)         },
                                  {0 ,  sin(phi)/cos(theta) ,  cos(phi)/cos(theta) }};

   x.subvec(1,3) =  x.subvec(1,3) +  (R_b2e*x.subvec(4,6))*delta_t; // euler_dot = R_b2e*[w_x w_y w_z]';  

  
   x[4] = 0;
   x[5] = 0;
   x[6] = w_z;
   
   
   // Position Equations
   double Ra2b[9];
   Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
   arma::mat Rn2r(Ra2b,3,3); // navigation to robot rotation matrix
   arma::mat::fixed<3,3> Rr2n = Rn2r.t();
   
   x.subvec(7,9) = x.subvec(7,9) + Rr2n*x.subvec(10,12)*delta_t;  
   
   x[10] = v_x;
   x[11] = 0;
   x[12] = 0;

   //------------------------------------
   // Update system covariance matrix P

   arma::mat::fixed<13,13> Jfx;
   arma::mat::fixed<13,2 > Jfu;
   
   //cout << x << endl;
  // Jac_euler_noise_driven_model(x, delta_t, Tau_r, Tau_p, Jfx, Jfu);

   Jac_euler_diff_driven_model(x, delta_t, r,L, Jfx, Jfu);

   

    arma::mat U;
    U.zeros(2,2);
    U.at(0,0) = pow(PAR.Robot_diff_Sigma_w,2);
    U.at(1,1) = pow(PAR.Robot_diff_Sigma_w,2);

    //cout << Jfx << endl;
    //    cout << Jfu << endl;
    //    cout << P << endl;


    int x_len = x.size();
    if(x_len > PAR.Robot_state_size)
    {   
        arma::mat Pdot;
        Pdot.zeros(x_len,x_len);
        Pdot(arma::span(0,12),arma::span(13,x_len-1)) = Jfx*P(arma::span(0,12),arma::span(13,x_len-1)) + P(arma::span(0,12),arma::span(13,x_len-1)); 
        Pdot(arma::span(13,x_len-1),arma::span(0,12)) = Pdot(arma::span(0,12),arma::span(13,x_len-1)).t();
        Pdot(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12)) + P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        
        P = P + Pdot*delta_t;

        //P(arma::span(0,12),arma::span(13,x_len-1)) = Jfx*P(arma::span(0,12),arma::span(13,x_len-1)); 
        //P(arma::span(13,x_len-1),arma::span(0,12)) = P(arma::span(0,12),arma::span(13,x_len-1)).t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + U;
    }
    else
    {
        arma::mat Pdot = Jfx*P(arma::span(0,12),arma::span(0,12)) + P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();

        //P(arma::span(0,12),arma::span(0,12)) =  Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + U;
        
        P(arma::span(0,12),arma::span(0,12)) =  P(arma::span(0,12),arma::span(0,12)) + Pdot*delta_t;

       
    }

     //cout << "x, std: " << sqrt(P(7,7)) << " y, std: " << sqrt(P(8,8)) <<  endl;
     int q = 10;


}


//-----------------------------------------------------------------------------
void EKF::prediction_euler_noise_driven(double delta_t)
{

    double sigma_w = PAR.Sigma_w;
    double sigma_a = PAR.Sigma_a;
    double Tau_r = PAR.Tau_r;
    double Tau_p = PAR.Tau_p;
    double kr = 1/Tau_r;
    double kp = 1/Tau_p; 

    // Attitude Equations
    double phi = x(1);
    double theta = x(2);
    double psi = x(3);
    // body rotational velocities to euler velocities Rotation matrix    
   
   arma::mat::fixed<3,3> R_b2e = {{1 ,  sin(phi)*tan(theta) ,  cos(phi)*tan(theta) },
                                  {0 ,       cos(phi)       ,    -sin(phi)         },
                                  {0 ,  sin(phi)/cos(theta) ,  cos(phi)/cos(theta) }};

   x.subvec(1,3) =  x.subvec(1,3) +  (R_b2e*x.subvec(4,6))*delta_t; // euler_dot = R_b2e*[w_x w_y w_z]';  

   x.subvec(4,6) = x.subvec(4,6) +  (-kr*x.subvec(4,6))*delta_t;

   // Position Equations
   double Ra2b[9];
   Euler_to_Ra2b_colum_major(phi, theta, psi, Ra2b);
   arma::mat Rn2r(Ra2b,3,3); // navigation to robot rotation matrix
   arma::mat::fixed<3,3> Rr2n = Rn2r.t();

   x.subvec(7,9) = x.subvec(7,9) + Rr2n*x.subvec(10,12)*delta_t;

   x.subvec(10,12) = x.subvec(10,12) +  (-kp*x.subvec(10,12))*delta_t;
   
   //------------------------------------
   // Update system covariance matrix P

   arma::mat::fixed<13,13> Jfx;
   arma::mat::fixed<13,6> Jfu;
   
   //cout << x << endl;
   Jac_euler_noise_driven_model(x, delta_t, Tau_r, Tau_p, Jfx, Jfu);

    arma::mat U;
    U.zeros(6,6);
    U(arma::span(0,2),arma::span(0,2)) = arma::eye(3,3)*pow(sigma_w*delta_t,2);
    U(arma::span(3,5),arma::span(3,5)) = arma::eye(3,3)*pow(sigma_a*delta_t,2);

    //cout << Jfx << endl;
    //    cout << Jfu << endl;
    //    cout << P << endl;


    int x_len = x.size();
    if(x_len > PAR.Robot_state_size)
    {   
      //  arma::mat tp = P(arma::span(0,12),arma::span(13,x_len));
      //  arma::mat tj = Jfx*P(arma::span(0,12),arma::span(13,x_len));
        P(arma::span(0,12),arma::span(13,x_len-1)) = Jfx*P(arma::span(0,12),arma::span(13,x_len-1)); 
        P(arma::span(13,x_len-1),arma::span(0,12)) = P(arma::span(0,12),arma::span(13,x_len-1)).t();
        P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + U;
    }
    else
    {
        
        P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + U;
        int q = 10;
    }

    

}
/*
% Initial States ***************************************************
% x meaning
% index  0  1  2  3  4   5   6   7  8  9  10  11  12  
%       q1 q2 q3 q4 w_x w_y w_z  x  y  z  v_x v_y v_z 
% Attitude states
% x(0:3)=   [q1 q2 q3 q4] -> quaternion orientation  (navigation to camera)
% x(4:6)=   [w_x w_y w_z ] ->  vel rotation in the body frame
% Position states
% x(7:9)= [x  y  z]  ->  Position in the navigation coordinate frame
% (navigation to camera)
% x(10:12)= [v_x v_y v_z]  -> Velocity in navigation coordinate frame.*/

//---------------------------------------------------------------------------------
void EKF::prediction_quat_noise_driven(double delta_t)
{
   
    double sigma_w = PAR.Sigma_w;
    
    double sigma_a = PAR.Sigma_a;        
    double Tau_r = PAR.Tau_r;
    double Tau_p = PAR.Tau_p;

    double kr = 1/Tau_r;
    double kt = 1/Tau_p;   
   
    // Attitude equations
    arma::vec w_u = x.subvec(4,6);
    arma::vec W = w_u*delta_t/2;
    double w = arma::norm(W);

    double sinwow;

    if (w==0)
    {
        sinwow = 1;
    }
    else
    {
        sinwow = sin(w)/w;
    }

    arma::mat W_mat = { {0, -W[0], -W[1], -W[2]},
                        {W[0], 0, -W[2], W[1]},
                        {W[1], W[2], 0, -W[0]},
                        {W[2], -W[1], W[0], 0} };    

    arma::mat M = cos(w)*arma::eye(4,4) + W_mat*sinwow;
    
    x.subvec(0,3) = M*x.subvec(0,3); // q_(k+1) = M*q_(k)
    x.subvec(0,3) = x.subvec(0,3)/arma::norm(x.subvec(0,3)); // normalize quaternion    

    x.subvec(4,6) = (arma::eye(3,3)-arma::eye(3,3)*kr*delta_t)*x.subvec(4,6);  // w_(k+1) = (I - I*kr*delta_t)*w_(k)
    //x.subvec(4,6) = x.subvec(4,6);

    // position equations
    x.subvec(7,9) = x.subvec(7,9) + x.subvec(10,12)*delta_t;

    x.subvec(10,12) = (arma::eye(3,3)-arma::eye(3,3)*kt*delta_t)*x.subvec(10,12);  // v_(k+1) = (I - I*kt*delta_t)*v_(k)

    //----------------------------------------------------------------------------------------------------------------

   //------------------------------------
    // Update system covariance matrix P
    
    double JFx[169];
    double JFu[78];
    double k[2] = {kr,kt};    
    
    std::vector<double> x_t;

    x_t = arma::conv_to<vec_t>::from(x.subvec(0,12)); // convert from armadillo vector to c++ vector
     
    arma::mat::fixed<13,13> Jfx;
    arma::mat::fixed<13,6> Jfu;

    Jac_quat_noise_driven_model2(x, delta_t, Tau_r, Tau_p, Jfx, Jfu);
    
    //Jac_noise_driven_model(&x_t[0], delta_t,k, JFx, JFu); // get Jacobians
    //arma::mat Jfx(JFx,13,13); // initialize armadillo matrix from array.
    //arma::mat Jfu(JFu,13,6); 

     
    /*
    arma::mat U;
    U.zeros(13,13);
    U(arma::span(0,3),arma::span(0,3)) = arma::eye(4,4)*pow(sigma_q*delta_t,2);
    U(arma::span(4,6),arma::span(4,6)) = arma::eye(3,3)*pow(sigma_w*delta_t,2);    
    U(arma::span(7,9),arma::span(7,9)) = arma::eye(3,3)*pow(sigma_p*delta_t,2);
    U(arma::span(10,12),arma::span(10,12)) = arma::eye(3,3)*pow(sigma_a*delta_t,2);
    */
    
    arma::mat U;
    U.zeros(6,6);
    U(arma::span(0,2),arma::span(0,2)) = arma::eye(3,3)*pow(sigma_w*delta_t,2);
    U(arma::span(3,5),arma::span(3,5)) = arma::eye(3,3)*pow(sigma_a*delta_t,2);

    

    int x_len = x.size();
    if(x_len > PAR.Robot_state_size)
    {   
      //  arma::mat tp = P(arma::span(0,12),arma::span(13,x_len));
      //  arma::mat tj = Jfx*P(arma::span(0,12),arma::span(13,x_len));
        P(arma::span(0,12),arma::span(13,x_len-1)) = Jfx*P(arma::span(0,12),arma::span(13,x_len-1)); 
        P(arma::span(13,x_len-1),arma::span(0,12)) = P(arma::span(0,12),arma::span(13,x_len-1)).t();
        P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + U;
    }
    else
    {
        P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + Jfu*U*Jfu.t();
        //P(arma::span(0,12),arma::span(0,12)) = Jfx*P(arma::span(0,12),arma::span(0,12))*Jfx.t() + U;
    }
    
    //cout << x << endl;
    //cout << P << endl;
}

//-------------------------------------------------------------------------------------------------------------------------------

void EKF::prediction_compute_CL_cov(double delta_t)
{
   
   if( max(abs(x(10)),abs(x(11))) > PAR.CL_min_vel_cl_uncertainty_update)
   {
     arma::mat::fixed<2,2>  Q; 
     Q ={ {pow(PAR.CL_Sigma_x,2), 0},                       
          {0, pow(PAR.CL_Sigma_y,2)} };

    Pc = Pc + Q*delta_t;
   }

   Cl_radius_uncertainty = std::sqrt( std::max(Pc(0,0),Pc(1,1)) ) ;
    //cout << max(abs(x(10)),abs(x(11))) << endl;
    //cout << "Pc_x: " << sqrt(Pc(0,0)) << " Pc_y: " << sqrt(Pc(1,1)) <<  endl;
}