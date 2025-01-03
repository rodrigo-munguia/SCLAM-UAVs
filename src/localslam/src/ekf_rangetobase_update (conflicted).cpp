#include "ekf.hpp"
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


void EKF::Rangetobase_update(RANGETOBASE &rangetobase)
{
   static bool init = false;
    
    //double predicted_range = std::sqrt(std::pow(x[7], 2) + std::pow(x[8], 2) + std::pow(x[9], 2));
   
    double predicted_range = std::sqrt( std::pow(x[7], 2) + std::pow(x[8], 2) );
     
    double z =  rangetobase.range;   
    double h =  predicted_range;
     
    
    cout << "Range to base: " << z << " " << h << " " << (z-h) << endl;
    //cout << "Altitude: " << x[9] <<  endl;

    
    int x_len = x.size();
    arma::mat H;
    // Form jacobian
    H.resize(1,x_len);
    //hr = [x/(x^2 + y^2 + z^2)^(1/2), y/(x^2 + y^2 + z^2)^(1/2), z/(x^2 + y^2 + z^2)^(1/2)]
    double den = predicted_range;    
    H(0,7) =  x(7)/den;
    H(0,8) =  x(8)/den;
    //H(0,9) =  x(9)/den;    
           
    double r = PAR.Sigma_range_update;        

    arma::mat H_P = H*P;

    arma::mat S = H_P*H.t() + r*r; // Innovation matrix
    //arma::mat K = P*H.t()*arma::inv_sympd(S);
    arma::mat K = P*H.t()*arma::inv(S); // Kalman gain
    
    

    if(init == false) 
    {
        arma::vec xp = x;
        xp = xp + K*(z-h);  // System vector update
        if(~std::isnan(xp[7]))
        {
            init = true;
        }
        else
        {
            
        }

    }
    else
    {  

        cout << "Range to base update gives nan values" << endl;
         x = x + K*(z-h);  // System vector update

        P = P - K*H_P;  // System Covariance matrix update 

    }

   

    int q = 10;
    


}