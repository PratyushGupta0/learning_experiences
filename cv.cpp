#include<opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include<math.h>
#include <tf/tf.h>
#include <ros/ros.h>


std::vector<double> PControl(double x, double y, double z ,double constant,double threshold)
{
    //If the drone has not changed orientation, then you can take coordinates in the drone frame itself
    //to get the magnitudes of the velocities, just be careful with the orientation of the axes.
    // This function will return values in the ground frame.
    std::vector<double> out_vec;
    double mod=x*x+y*y+z*z;
    if(mod>=threshold){
    out_vec.push_back(constant*z);
    out_vec.push_back(-1*constant*x);
    out_vec.push_back(-1*constant*y);}
    else{
    out_vec.push_back((constant*z)/mod);
    out_vec.push_back((-1*constant*x)/mod);
    out_vec.push_back((-1*constant*y)/mod);}
    return out_vec;
}
std::vector<double> coord_wrt_bot(double center_x,double center_y, double distance, double img_center_x,double img_center_y)
{
    std::vector<double> out_vec; //(X,Y,Z) will be the output format
    
    double x_angle=((center_x-img_center_x)/(2*img_center_x))*1.39;
    double y_angle=((center_y-img_center_y)/(2*img_center_y))*1.39;
    double Z=distance/(sqrt(1+tan(x_angle)*tan(x_angle)+tan(y_angle)*tan(y_angle)));
    out_vec.push_back(Z*tan(x_angle));
    out_vec.push_back(Z*tan(y_angle));
    out_vec.push_back(Z);

    return out_vec;

}


// int main(){
tf2::Vector3 transform_vector(double vx,double vy,double vz, double qx,double qy,double qz, double qw)
{
 

  tf2::Quaternion q; // x, y, z, w in order
  q[0]=qx;
  q[1]=qy;
  q[2]=qz;
  q[3]=qw;

  tf2::Vector3 original;
  original[0]=vx;
  original[1]=vy;
  original[2]=vz;

  tf2::Vector3 axis = q.getAxis(); // Get the rotation axis of the quaternion
  double angle = q.getAngle(); //Get the rotation angle of the quaternion

  
  return original.rotate(q.getAxis(), q.getAngle());
}
int main()
{
    
    return 0;
}