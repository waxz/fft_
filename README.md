1.[fftw](https://github.com/waxz/fftw)

http://www.fftw.org/download.html

how to install

    make clean
    ./configure --enable-shared
    make
    sudo make install
    sudo ln -s /usr/lib/x86_64-linux-gnu/libfftw3_omp.so.3 /usr/lib/libfftw3_omp.so
     
add flag ./configure --enable-shared  for bug 
/usr/bin/ld: /usr/local/lib/libfftw3f.a(mapflags.o): relocation R_X86_64_32 against `.rodata' can not be used when making a shared object; recompile with -fPIC
/usr/local/lib/libfftw3f.a: error adding symbols: Bad value

    
2. [fftw++](https://github.com/dealias/fftwpp) , c++ api for fftw

http://fftwpp.sourceforge.net/


