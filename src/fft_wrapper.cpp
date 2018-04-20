//
// Created by waxz on 18-4-11.
//

#include <iostream>


#include <valarray>
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>

#include <iostream>
#include <istream>
#include <ostream>
#include <string>
#include <exception>
#include <fftw++/fftw++.h>
#include "fftw++/Array.h"
#include <fft_wrapper/fft_wrapper.h>
#include <math_util/sort.h>
using  namespace std;
using namespace utils;
using namespace Array;
using namespace fftwpp;

void fft_1d(valarray<float> signal,  valarray<double> &real,  valarray<double> &imag){
    cout << "1D real to complex out-of-place FFT" << endl;

    fftw::maxthreads=get_max_threads();

    unsigned int n=signal.size();
    unsigned int np=n/2+1;
    size_t align=sizeof(Complex);

    real = valarray<double>(1,np);
    imag = valarray<double>(1,np);

    array1<Complex> F(np,align);
    array1<double> f(n,align);               // For out-of-place transforms
//  array1<double> f(2*np,(double *) F()); // For in-place transforms

    rcfft1d Forward(n,f,F);
    crfft1d Backward(n,F,f);

    for(unsigned int i=0; i < n; i++) f[i]=signal[i];

//    cout << endl << "input:" << endl << f << endl;

    Forward.fft(f,F);

//    cout << endl << "output:" << endl << F << endl;
    for(unsigned int i=0; i < np; i++) {
        real[i]=F[i].real();
        imag[i]=F[i].imag();
    }
    real /= np;
    imag /= np;

    printf("fft first element %f,%f",real[1],imag[1]);


    Backward.fftNormalized(F,f);


//    cout << endl << "back to input:" << endl << f << endl;
}

FFT_Fitter::FFT_Fitter(ros::NodeHandle nh, ros::NodeHandle nh_private): nh_(nh), nh_private_(nh_private) {
    gen_ptr_ = new Laser_Simulator(nh, nh_private);
    init_params();

}

void FFT_Fitter::transform(const sensor_msgs::LaserScan &scan,  geometry_msgs::Pose &pose) {

    if (scan_info_.ranges.empty()){
        scan_info_ = scan;
    }

    // scan to valarray
    ROS_ASSERT(scan.ranges.size() > 0);
    scan_sens_ = wn::vector_valarray<float>(scan.ranges);
    ROS_ASSERT(scan_sens_.size() > 0);


    for (int i=0;i<iteration_;i++){
        loop(pose);
        if (signal_diff_ < final_diff_)
            break;
    }

}

void FFT_Fitter::loop(geometry_msgs::Pose &pose) {
    // pose to scan

    // diff signal

    // transform signal

    // update  pose
    sensor_msgs::LaserScan::Ptr map_scan_ptr = gen_ptr_->get_laser(pose);
    ROS_ASSERT(map_scan_ptr->ranges.size() > 0);
    scan_ref_ = wn::vector_valarray<float>(map_scan_ptr->ranges);
    signal_ = scan_ref_ - scan_sens_;
    adaptive_remove();
    if (signal_diff_ < final_diff_)
        return;

    fft_1d(signal_, real_, imag_);
//    double N = real_.size();
//    real_ = real_/N;
//    imag_ = imag_/N;
//
    diff_x = real_[1];
    diff_y = imag_[1];
    rad = atan2(imag_[1], real_[1]);

    pose.position.x -= diff_x;
    pose.position.y += diff_y;

}

void FFT_Fitter::adaptive_remove() {
    // remove point difference > e, with adaptive
    valarray<float> abs_diff = abs(signal_);
    wn::sort<float >(abs_diff);
    float low = abs_diff[int(low_end_*abs_diff.size())];
    float high = abs_diff[int(high_end_*abs_diff.size())];
    valarray<float> slect_diff = (abs_diff[(abs_diff>low) && (abs_diff < high) ]);
    //float diff_ave = outlier_max_*slect_diff.sum()/float(slect_diff.size());
    float diff_ave = (0.5*outlier_max_)*(low+ high);

    signal_ = valarray<float>(signal_[(abs(signal_)<diff_ave ) && (scan_ref_ < scan_info_.range_max) && (scan_sens_ < scan_info_.range_max)]);
    if (signal_.size() < valid_per_*scan_info_.ranges.size()){
        signal_diff_ = 0.0;
        return;
    }

    signal_diff_ = (abs(signal_)).sum()/(signal_.size());

    printf("\n signal_ length %d\n",int(signal_.size()));
}

void FFT_Fitter::get_base_pose(const sm::LaserScan &sensor_scan, gm::Pose &base_pose,
                               const tf::Transform &base_laser_tf) {

    gm::Pose laser_pose;

    tf::Pose base_pose_tf, laser_pose_tf;
    tf::poseMsgToTF(base_pose, base_pose_tf);
    tf::poseTFToMsg(base_pose_tf*base_laser_tf,laser_pose);


    transform(sensor_scan,laser_pose);

    tf::poseMsgToTF(laser_pose, laser_pose_tf);
    tf::poseTFToMsg(laser_pose_tf*base_laser_tf.inverse(),base_pose);


}

void FFT_Fitter::init_params() {
    if (!nh_private_.getParam("low_end", low_end_))
        low_end_ = 0.25;
    if (!nh_private_.getParam("high_end_", high_end_))
        high_end_ = 0.75;
    if (!nh_private_.getParam("outlier_max", outlier_max_))
        outlier_max_ = 2.0;
    if (!nh_private_.getParam("final_diff", final_diff_))
        final_diff_ = 0.01;
    if (!nh_private_.getParam("iteration", iteration_))
        iteration_ = 4;
    if (!nh_private_.getParam("valid_per", valid_per_))
        valid_per_ = 0.5;

}