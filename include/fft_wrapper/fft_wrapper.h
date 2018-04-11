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
// vector to valarray
template<class T>
valarray<T> vector_valarray(const vector<T> &a) {
    const T *arr = &(a[0]);
    valarray<T> res(arr, a.size());
    return res;
}

template<class T>
valarray<T> valarray_vector(const valarray<T> &a) {
    const T *arr = &(a[0]);
    vector<T> res(arr, arr + a.size());
    return res;
}


void fft_1d(valarray<double> signal, valarray<double> &real, valarray<double> &imag);


class FFT_Fitter{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    Laser_Simulator *gen_ptr;
    valarray<double> real, imag;
    double  diff_x, diff_y, rad;

    valarray<float> scan_sens, scan_ref,signal;
    void loop(geometry_msgs::Pose &pose);

public:
    FFT_Fitter(ros::NodeHandle nh, ros::NodeHandle nh_private);
    void transform(const sensor_msgs::LaserScan &scan, geometry_msgs::Pose &pose);

};



#endif //FFT_WRAPPER_FFT_WRAPPER_H
