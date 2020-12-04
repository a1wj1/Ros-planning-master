#ifndef TRAJECTORY_OPTIMIZATION_H
#define TRAJECTORY_OPTIMIZATION_H

#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include <vector>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
struct Point3d_s
{
    double x;
    double y;
    double z;
};
struct Point4d_s
{
    double x;
    double y;
    double z;
    double o;
};
struct Point2d_s
{
    double x;
    double y;
};
class TrajectoryOptimization
{
public:
    TrajectoryOptimization();
    void test(void);
    void trajectory_start_cutting(geometry_msgs::PoseArray &path_points, geometry_msgs::PoseArray &path_after_cutting, int index, Point3d_s &target_pose, Point3d_s &set_pose);
    void trajectory_stop_cutting(geometry_msgs::PoseArray &path_pointss, geometry_msgs::PoseArray &path_after_cuttings, int indexs, Point3d_s &target_poses, Point3d_s &set_poses);
    void trajectory_same_cutting(geometry_msgs::PoseArray &path_points, geometry_msgs::PoseArray &start_path_points, geometry_msgs::PoseArray &stop_path_points, geometry_msgs::PoseArray &path_after_cutting, int index, Point3d_s &target_pose, Point3d_s &set_pose, int indexs, Point3d_s &target_poses, Point3d_s &set_poses);

    geometry_msgs::Pose point_posess;
    /**
     * @brief 梯形时间分配
     * @param path_points 路径点
     * @param vel 车辆速度
     * @param acc 车辆加速度
     * @param time_allo 输出时间分配
     * @return true 正确
     *         false 出错
     */
    bool acc_time_allocation(Eigen::MatrixXd &path_points, double vel, double acc, Eigen::MatrixXd &time_allo);

    void average_interpolation(Eigen::MatrixXd &input, Eigen::MatrixXd &output, double interval_dis, double distance);

private:
    int val_;
};

#endif