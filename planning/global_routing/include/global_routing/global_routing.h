#ifndef __GLOBAL_ROUTING_H
#define __GLOBAL_ROUTING_H

#include <iostream>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>

#include <global_routing/minimumjerk.h>
#include "trajectory_optimization.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <visualization_msgs/MarkerArray.h>
#include <boost/thread.hpp>

// msg
#include "planning_msgs/global_routing.h"
#include "planning_msgs/lanelets_id.h"
#include "planning_msgs/vehicle_pose.h"
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <lanelet2_extension/projection/mgrs_projector.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
namespace global_routing
{

    using namespace lanelet;

    struct Point3D_s
    {
        double x;
        double y;
        double z;
    };

    struct State_s
    {
        double x;
        double y;
        double z;
        double pitch;
        double roll;
        double yaw;
    };

    class GlobalRouting
    {
    public:
        GlobalRouting(void);
        ~GlobalRouting(void);
        void soultion(void);
        void soultion(long int start_id, long int end_id);
        void soultion(geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose);
        // lanelet
        void copy_with_high_definition_map(std::string &file_path, lanelet::GPSPoint &origin_gps_point);
        bool get_shortest_path(long int start_id, long int end_id);
        bool get_shortest_path_according_to_pose(geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose);
        void get_path_points();
        bool getClosestLanelet(const geometry_msgs::Pose &search_pose, const lanelet::LaneletMapPtr &lanelet_map_ptr_,
                               lanelet::Lanelet *closest_lanelet, double distance_thresh = 10.0);
        // generate trajectory
        void generate_trajectory(void);
        void generate_global_routing(routing::LaneletPath &shortestPath, geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose);
        int Find_nearest_point(const geometry_msgs::PoseArray &pose_array, Point3d_s &pose);
        void back_vehical(routing::LaneletPath &shortestPath, geometry_msgs::Pose &start_pose, geometry_msgs::Pose &goal_pose);
        bool isInLanes(const lanelet::Lanelet &lanelet, const lanelet::ConstPoint3d &point);
        bool is_direction_Valid(geometry_msgs::Pose &pose);
        constexpr double normalizeRadian(const double rad, const double min_rad, const double max_rad);
        // get trajectory path points 获得轨迹点
        geometry_msgs::PoseArray &getTrajPathArray(void);
        nav_msgs::Path &getTrajPath(void);
        nav_msgs::Path &getPath(void);
        geometry_msgs::PoseArray &getStorePathArray(void);
        geometry_msgs::PoseArray &getStorePathArrayAfterInterpolation(void);
        Eigen::MatrixXd &getMatrixPathPointAfterInterpolation(void);
        Eigen::MatrixXd &getMatrixPathPointafterInterpolation(void);
        Eigen::MatrixXd &gethdmap_way_points(void);
        visualization_msgs::Marker &getTrajPathMarker(void);
        planning_msgs::lanelets_id &getMapLaneletId(void);
        // thread's function
        void thread_generate_trajectory(void);
        void thread_routing(void);

        // ros
        // call back
        void goal_call_back(const planning_msgs::global_routing &msg);
        void odom_call_back(const nav_msgs::Odometry &odom);
        void start_pose_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
        void goal_pose_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg);
        // control
        int find_nearest_point_index(const geometry_msgs::PoseArray &pose_array, Point3d_s &pose);
        void control_vehicle(const geometry_msgs::PoseArray &trj_point_array, nav_msgs::Odometry &odom, int tar_index, double &out_turn_agl, int &out_index);
        bool vehical_stop(const geometry_msgs::PoseArray &trj_point_array, nav_msgs::Odometry &odom); //判断车是否到了路径的尽头
        void publish_vehical_start_pose(const geometry_msgs::Pose &start_pose);
        double VEHICLE_VEL;        //车速度
        double STANLEY_K;          //stanly算法的K值
        bool is_vehical_start_set; //判断车是否可以开始开车
        bool is_vehical_stop_set;  //判断车是否要停下来
        bool is_vehical_cango;     //判断车是否可行，不能逆向行使
        bool is_vehical_start;     //判断车起点
        bool is_vehical_stop;      //判断车终点
        autoware_lanelet2_msgs::MapBin msg;
        traffic_rules::TrafficRulesPtr trafficRules;
        traffic_rules::TrafficRulesPtr pedestrainRules;
        Lanelet start_lanelet;
        Lanelet goal_lanelet;

    private:
        // lanlet2
        // 地图文件
        std::string file_path_;
        // 地图
        LaneletMapPtr map_;

        // 路线图存储
        routing::RoutingGraphUPtr routingGraph_;
        routing::RoutingGraphUPtr pedestrGraph_;
        // 原点
        lanelet::GPSPoint origin_gps_point_;
        struct Point3D_s origin_utm_point_;
        // 路径
        routing::LaneletPath shortestPath_;
        ConstLanelet get_start_path;
        ConstLanelet get_stop_path;
        // 路径点
        geometry_msgs::PoseArray store_path_array_;
        geometry_msgs::PoseArray stores_path_array_;      //////////////
        geometry_msgs::PoseArray all_path_array_;         /////////////
        geometry_msgs::PoseArray start_path_array_;       //////////////////
        geometry_msgs::PoseArray stop_path_array_;        ////////////////
        geometry_msgs::PoseArray after_start_path_array_; //////////////////
        geometry_msgs::PoseArray after_stop_path_array_;  ////////////////
        geometry_msgs::PoseArray after_start_path_array;  //////////////////
        geometry_msgs::PoseArray after_stop_path_array;   ////////////////
        geometry_msgs::PoseArray test_path_array;         ////////////////

        nav_msgs::Path path_points_;
        Eigen::MatrixXd hdmap_way_points_;
        Eigen::MatrixXd hdmap_way_points;
        geometry_msgs::Pose point_pose;
        geometry_msgs::Pose point_posess; /////////////////////
        geometry_msgs::PoseStamped pose_stamp;
        Point3d_s start_pp;
        Point3d_s stop_pp;
        Point3d_s st_pp;
        Point3d_s sp_pp;
        // lanelet id
        planning_msgs::lanelets_id map_lanelet_id_; //返回车道id

        // 轨迹点
        nav_msgs::Path traj_path_;
        geometry_msgs::PoseArray traj_path_array_;
        geometry_msgs::PoseArray traj_path_array_after_interpolation_;
        visualization_msgs::Marker traj_path_marker_;
        Eigen::MatrixXd path_point_after_interpolation_;
        Eigen::MatrixXd path_point_after_interpolation;
        long int start_id_;
        long int end_id_;
        geometry_msgs::Pose start_pose_;
        geometry_msgs::Pose goal_pose_;
        bool is_start_pose_set;
        bool is_goal_pose_set;
        planning_msgs::vehicle_pose vehicle_start_pose_;

        // trajectory optimiazation
        TrajectoryOptimization traj_opti_;
        // minimum jerk
        MinimumJerk *polynomial_solver_;

        // ROS
        // visual pub
        ros::Publisher waypoints_pub_;
        ros::Publisher waypoints_vis_pub_;
        ros::Publisher marker_pub_;
        ros::Publisher lanelets_id_pub_;
        ros::Publisher control_data_pub_;
        ros::Publisher vehicle_start_pose_pub_;
        ros::Subscriber goal_sub_;
        ros::Subscriber odom_sub_;
        ros::Subscriber start_pose_subscriber_;
        ros::Subscriber goal_pose_subscriber_;
        ros::Subscriber bin_maps_sub;
        // thread
        boost::thread *routing_thread_;
        boost::thread *generate_trj_thread_;
        boost::recursive_mutex routing_mutex_;
        boost::recursive_mutex generate_trj_mutex_;

        // odom
        nav_msgs::Odometry sub_odom_;

        // status
        State_s car_state;

        // flag
        bool is_received_goal_;
        bool is_update_next_path_;
        bool is_begin_goal_;
    };

} // namespace global_routing

#endif