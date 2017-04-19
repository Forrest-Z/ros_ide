
#include <iostream>
#include "ros/ros.h"
#include "ros/time.h"
#include <boost/shared_ptr.hpp>
// #include "lidarpath.h"
#include "dwaplanner.h"
#include "curvemodel.h"
#include "dubins.h"
#include <ctime>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ivcbp");
    ros::NodeHandle nh;
    std::unique_ptr<curvemodel> curve(new curvemodel());
    std::unique_ptr<dwaplanner> dwa(new dwaplanner(nh));
    //   std::unique_ptr<lidarpath> lidarnav (new lidarpath(nh));
    ros::Rate loop(5.0);
    while (ros::ok())
    {
        ros::spinOnce();
        //     lidarnav->run();
//         curve->run();
        //     dwa->run();
        clock_t start, finish;
        double duration;
        start = clock();
        dwa->test();
        finish = clock();
        duration = (double)(finish - start) / CLOCKS_PER_SEC;
        // std::cout<<"The algorithm costs: "<<duration<<" seconds!"<<std::endl;
        loop.sleep();
    }
    return 0;
}