//
// Created by waxz on 18-4-11.
//

#ifndef FFT_WRAPPER_FFT_WRAPPER_H
#define FFT_WRAPPER_FFT_WRAPPER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <laser_simulator/laser_simulator.h>
#include <vector>
#include <valarray>
using  namespace std;



void fft_1d(valarray<double> signal, valarray<double> &real, valarray<double> &imag);


class FFT_Fitter{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    Laser_Simulator *gen_ptr_;
    valarray<double> real_, imag_;
    double  diff_x, diff_y, rad;

    sensor_msgs::LaserScan scan_info_;

    valarray<float> scan_sens_, scan_ref_,signal_;
    void loop(geometry_msgs::Pose &pose);
    void adaptive_remove();

public:
    FFT_Fitter(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void transform(const sensor_msgs::LaserScan &scan, geometry_msgs::Pose &pose);

};



#endif //FFT_WRAPPER_FFT_WRAPPER_H
