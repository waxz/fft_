//
// Created by waxz on 18-4-12.
//

#ifndef FFT_WRAPPER_SORT_H
#define FFT_WRAPPER_SORT_H
#include <valarray>
#include <vector>
#include <string>
using namespace std;
namespace wn{
    // vector to valarray
    template<class T>
    valarray<T> vector_valarray(const vector<T> &a) {
        const T *arr = &(a[0]);
        valarray<T> res(arr, a.size());
        return res;
    }

    // valarray to vector
    template<class T>
    vector<T> valarray_vector(const valarray<T> &a) {
        const T *arr = &(a[0]);
        vector<T> res(arr, arr + a.size());
        return res;
    }

    // sort
    template <class T>
    void sort(vector<T> &a, string order = "<") {
        if (order == "<"){
            sort(a.begin(),a.end());
        } else if (order == ">"){
            sort(a.rbegin(),a.rend());
        }
    }

    template <class T>
    void sort(valarray<T> &a, string order = "<") {
        // valarray to to vector
        // sort vector
        // vector to valarray
        vector<T> vec = valarray_vector<T>(a);
        if (order == "<"){
            sort(vec.begin(),vec.end());

        } else if (order == ">"){
            sort(vec.rbegin(),vec.rend());
        }
        a = vector_valarray<T>(vec);
    }


}
#endif //FFT_WRAPPER_SORT_H
