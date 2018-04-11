cmake_minimum_required(VERSION 2.8)
project(fft_wrapper)
set(CMAKE_CXX_STANDARD 98)

find_package(catkin REQUIRED COMPONENTS
        cmake_modules roscpp
        )
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS roscpp
)

include_directories(
        include
        src
        ${catkin_INCLUDE_DIRS} )

add_library(${PROJECT_NAME} src/fft_wrapper.cpp include/fftw++/fftw++.cc)
target_link_libraries(${PROJECT_NAME}  fftw3 fftw3_omp)




install(
        TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(${PROJECT_NAME}_node src/main.cpp  )
target_link_libraries(${PROJECT_NAME}_node  ${PROJECT_NAME}  )