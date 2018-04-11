1.fftw

http://www.fftw.org/download.html

how to install

    ./configure --enable-shared
    make
    sudo make install
     
add flag ./configure --enable-shared  for bug 

    /usr/bin/ld: /usr/local/lib/libfftw3f.a(mapflags.o): relocation R_X86_64_32 against `.rodata' can not be used when making a shared object; recompile with -fPIC
    /usr/local/lib/libfftw3f.a: error adding symbols: Bad value

2. fftw++ , c++ api for fftw

http://fftwpp.sourceforge.net/


