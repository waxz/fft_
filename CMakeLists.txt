cmake_minimum_required(VERSION 2.8)
project(fft_wrapper)
if (NOT CMAKE_VERSION VERSION_LESS 3.1)
    set(CMAKE_CXX_STANDARD 11)
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif ()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -pthread -g -O0 -fprofile-arcs -ftest-coverage")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic -pthread -g -O0 -fprofile-arcs -ftest-coverage")
# Build warning with -pedantic https://github.com/ros/rosconsole/issues/9
# example how to set c++ compiler flags for GNU
if (CMAKE_CXX_COMPILER_ID MATCHES GNU)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-unknown-pragmas -Wno-sign-compare -Woverloaded-virtual -Wwrite-strings -Wno-unused")
    set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage")
endif ()

find_package(catkin REQUIRED COMPONENTS
        cmake_modules roscpp laser_simulator
        )
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp laser_simulator
)

include_directories(
        include
        src
        ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} src/fft_wrapper.cpp include/fftw++/fftw++.cc)
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} fftw3 fftw3_omp)




install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(${PROJECT_NAME}_node src/main.cpp  )
target_link_libraries(${PROJECT_NAME}_node  ${PROJECT_NAME}  )