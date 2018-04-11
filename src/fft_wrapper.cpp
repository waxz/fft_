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
#include <fft_wrapper/fft_wrapper.h>

using  namespace std;
using namespace utils;
using namespace fftwpp;

void fft_1d(valarray<float> signal,  valarray<double> &real,  valarray<double> &imag){
    //fftw
    cout << "1D real to complex out-of-place FFTs not using the Array class"
         << endl;

    fftw::maxthreads=get_max_threads();

    unsigned int n=signal.size();
    unsigned int np=n/2+1;
    real = valarray<double> (1,np);
    imag = valarray<double> (1,np);
    double *f=FFTWdouble(n);
    Complex *g=FFTWComplex(np);

    rcfft1d Forward(n,f,g);
    crfft1d Backward(n,g,f);

    for(unsigned int i=0; i < n; i++) f[i]=signal[i];

    cout << "\ninput (" << n << " doubles):" << endl;
    for(unsigned int i=0; i < n; i++) cout << f[i] << (i!=n-1 ? " " : "\n");

    Forward.fft(f,g);

    cout << "\noutput (" << np << " complex):" << endl;
    for(unsigned int i=0; i < np; i++)
    {
        real[i] = g[i].real();
        imag[i] = g[i].imag();


        cout << g[i] <<  (i!=np-1 ? " " : "\n");
    }


    Backward.fftNormalized(g,f);

    cout << "\ntransformed back:" << endl;
    for(unsigned int i=0; i < n; i++) cout << f[i] <<  (i!=n-1 ? " " : "\n");

    deleteAlign(g);
    deleteAlign(f);
}

FFT_Fitter::FFT_Fitter(ros::NodeHandle nh, ros::NodeHandle nh_private): nh_(nh), nh_private_(nh_private) {
    gen_ptr = new Laser_Simulator(nh, nh_private);

}

void FFT_Fitter::transform(const sensor_msgs::LaserScan &scan,  geometry_msgs::Pose &pose) {

    // scan to valarray
    scan_sens = vector_valarray<float>(scan.ranges);

    for (int i=0;i<4;i++){
        loop(pose);
    }

}

void FFT_Fitter::loop(geometry_msgs::Pose &pose) {
    // pose to scan

    // diff signal

    // transform signal

    // update  pose
    sensor_msgs::LaserScan::Ptr map_scan_ptr = gen_ptr->get_laser(pose);
    scan_ref = vector_valarray<float>(map_scan_ptr->ranges);
    signal = scan_ref - scan_sens;
    fft_1d(signal, real, imag);
    double N = real.size();
    real = real/N;
    imag = imag/N;

    diff_x = real[1];
    diff_y = imag[1];
    rad = atan2(imag[1], real[1]);

    pose.position.x += diff_x;
    pose.position.y += diff_y;

}