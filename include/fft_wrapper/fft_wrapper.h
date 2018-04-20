//
// Created by waxz on 18-4-11.
//

#ifndef FFT_WRAPPER_FFT_WRAPPER_H
#define FFT_WRAPPER_FFT_WRAPPER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
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
    void init_params();
    double  diff_x, diff_y, rad, signal_diff_;
    float low_end_, high_end_, outlier_max_,final_diff_, valid_per_;
    int iteration_;

    sensor_msgs::LaserScan scan_info_;

    valarray<float> scan_sens_, scan_ref_,signal_;
    void loop(geometry_msgs::Pose &pose);
    void adaptive_remove();

public:
    FFT_Fitter(ros::NodeHandle nh, ros::NodeHandle nh_private);
    // call fftw
    void transform(const sensor_msgs::LaserScan &scan, geometry_msgs::Pose &pose);
    // wrap fftw, and transform base_pose
    void get_base_pose(const sm::LaserScan &sensor_scan, gm::Pose &init_pose,
                       const tf::Transform &base_laser_tf);

    // compute match error, check amcl match accurucy, csm accurucy



};



#endif //FFT_WRAPPER_FFT_WRAPPER_H
