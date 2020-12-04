#include "global_routing_node.h"
#include "global_routing.h"
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread.hpp>
#include "trajectory_optimization.h"
// msgs
#include <planning_msgs/global_routing.h>
#include <planning_msgs/lanelets_id.h>

using namespace global_routing;

// global routing
GlobalRouting *G_global_routing;

int main(int argc, char **argv)
{
    //std::cout<<"------------------- global_routing_node -------------------"<<std::endl;
    ros::init(argc, argv, "global_routing");
    ros::NodeHandle n;
    // 指定循环德频率 10ms
    ros::Rate loop_rate(100);
    int loop_ten_times = 10;
    uint32_t loop_count = 0;
    // //std::cout<<"------------------- ------------------ -------------------"<<std::endl;
    // //创建对象，调用构造函数
    G_global_routing = new GlobalRouting;
    // /////////////////////////////////////////// TEST ////////////////////////////////////////////
    std::string file_path;
    double_t ori_lat, ori_lon;
    int vel, acc;
    while (ros::ok())
    {
        // loop_count++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}