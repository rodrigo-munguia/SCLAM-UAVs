/*---------------------------------------------------
Rodrigo Munguía 2021.

System parameters
-----------------------------------------------------
*/
#include <string>

#ifndef PARAMETERS_H
#define PARAMETERS_H

using namespace std;



struct parameters
{   
    
    double Sigma_roll;    
    double Sigma_pitch;
    double Sigma_yaw;

    double Sigma_Alt;
    double Sigma_Spd;
    
    vector<double> Base_location;
    double Sigma_rangetobase;
    

} ;   





#endif /*PARAMETERS_H*/