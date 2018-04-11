#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


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

using  namespace std;
using namespace utils;
using namespace fftwpp;



using std::cout;
using std::endl;


void find_valid(){
    // get normal diff
    // assume direction is correct
    // get all point's diff
    // sort, remove extreme value
    // get diff thresh
    // set valid value to each point

}

//
void fft_loop(){
    // refine result with loop

    // maybe loop 3 times

    // if final error converge , consider fft successful


}


int main() {



    //fftw
    cout << "1D real to complex out-of-place FFTs not using the Array class"
         << endl;




    fftw::maxthreads=get_max_threads();

    unsigned int n=4;
    unsigned int np=n/2+1;
    double *f=FFTWdouble(n);
    Complex *g=FFTWComplex(np);

    rcfft1d Forward(n,f,g);
    crfft1d Backward(n,g,f);

    for(unsigned int i=0; i < n; i++) f[i]=i;

    cout << "\ninput (" << n << " doubles):" << endl;
    for(unsigned int i=0; i < n; i++) cout << f[i] << (i!=n-1 ? " " : "\n");

    Forward.fft(f,g);

    cout << "\noutput (" << np << " complex):" << endl;
    for(unsigned int i=0; i < np; i++) cout << g[i] <<  (i!=np-1 ? " " : "\n");

    Backward.fftNormalized(g,f);

    cout << "\ntransformed back:" << endl;
    for(unsigned int i=0; i < n; i++) cout << f[i] <<  (i!=n-1 ? " " : "\n");

    deleteAlign(g);
    deleteAlign(f);







    return 0;
}