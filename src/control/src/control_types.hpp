#ifndef CONTROL_TYPES_H
#define CONTROL_TYPES_H

/*---------------------------------------------------
Rodrigo Munguía 2023.


-----------------------------------------------------
*/
//#include "parameters.hpp"

using namespace std;
using namespace cv;



// commands structure
struct CMD
{
    string name;
    vector<double> args;     
};


struct ROBOT_STATE
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    double vx;
    double vy;
    double vz;
    double vyaw;
    double r_u; // robot uncertanty
};







#endif
