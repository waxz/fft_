#include <iostream>



#include <fft_wrapper/fft_wrapper.h>
#include <ros/ros.h>
using  namespace std;




using std::cout;
using std::endl;


Laser_Simulator *gen_ptr;

int main(int argc, char **argv) {
    ros::init(argc, argv, "fft_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // why new pointer ok; normal constructor fail with
    // "Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)"
    Laser_Simulator gen(nh, nh_private);




//    Laser_Simulator gen(nh, nh_private);


    FFT_Fitter fft_fitter(nh, nh_private);



    // todo test
    geometry_msgs::Pose pose;
    pose.position.x = 2;
    pose.position.y = 3;
    pose.position.z = 0.29;
    // must fill orientation.w
    pose.orientation.w = 1.0;

    sensor_msgs::LaserScan scan_sens = gen.get_laser(pose);

    pose.position.x = 2.4;
    pose.position.y = 3.3;
    pose.position.z = 0.29;

    fft_fitter.transform(scan_sens,pose);
//
    printf("fft pose %f,%f",pose.position.x,pose.position.y);





    return 0;
}
