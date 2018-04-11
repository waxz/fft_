#include <iostream>



#include <fft_wrapper/fft_wrapper.h>

using  namespace std;




using std::cout;
using std::endl;


int main() {
    valarray<double > signal(1,4);
    for (int i=0;i<4;i++){
        signal[i] = i;
    }
    valarray<double> real, imag;

    fft_1d(signal, real, imag);

    cout<<real[0];











    return 0;
}