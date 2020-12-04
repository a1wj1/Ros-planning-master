#ifndef MINIMUMJERK_H
#define MINIMUMJERK_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
#include <vector>
#include <deque>
#include <boost/format.hpp>

class MinimumJerk
{
public:
    MinimumJerk();
    MinimumJerk(Eigen::MatrixXd &path_points_3d);
    MinimumJerk(Eigen::MatrixXd &path_points_3d, Eigen::MatrixXd &T_allocation);
    Eigen::MatrixXd compute_Q(int n, int r, float t1, float t2);

    /**
     * @brief insert matrix diagonal block,size of each block is equal
     * @param result
     * @param block
     * @param block_pos
     * @return
     */
    bool insert_matrix_diagonal_block(Eigen::MatrixXd &result, Eigen::MatrixXd &block, const int block_pos);

    Eigen::MatrixXd compute_C_transpose(int n);
    /**
     * @brief MinimumJerk::closed_form_solution
     *        多项式阶次  5次多项式，6个系数
     * @param waypoints  一维列向量，对应一维的位置点
     * @param result
     * @return
     */
    bool closed_form_solution(Eigen::MatrixXd &waypoints, Eigen::MatrixXd &T_allocation, Eigen::MatrixXd &result);

    /**
     * @brief MinimumJerk::closed_form_solution_3D_with_time_allocation
     *        求解三维空间中曲线 三维曲线
     *        多项式阶次  5次多项式，6个系数
     * @param waypoints 三维列向量，对应三维的位置点
     * @param result
     * @return
     */
    bool closed_form_solution_3D_with_time_allocation(Eigen::MatrixXd &waypoints_xyz, Eigen::MatrixXd &T_allocation, Eigen::MatrixXd &result);

    /**
     * @brief 求解函数
     * @param path_points_3d 三维的路径点
     * @param T_allocation   每段轨迹的时间分配
     */
    void solution(Eigen::MatrixXd &path_points_3d, Eigen::MatrixXd &T_allocation);

    /**
     * @brief 计算五阶多项式
     * @param a 多项式系数，大小：6x1
     * @param t t时刻
     * @return 多项式的结果
     */
    double polynomail_degree_5(Eigen::MatrixXd &a, double t);

    /**
     * @brief get_waypoints
     * @param time_allocation 时间分配
     * @return void
     */
    void get_waypoints(Eigen::MatrixXd &time_allocation);

    /**
     * @brief 发布路点
     * @param void
     * @return void
     */
    void publish_waypoints(void);

    /**
     * @brief 获取轨迹的点
     * @param void
     * @return nav_msgs::Path
     */
    nav_msgs::Path get_trajectory_points(void);

    /**
     * @brief get_trajectory_points_array
     * @param void
     * @return geometry_msgs::PoseArray
     */
    geometry_msgs::PoseArray get_trajectory_points_array(void);

    /**
     * @brief set_origin_xyz
     * @param x 偏移坐标x
     * @param y 偏移坐标y
     * @param z 偏移坐标z
     * @return geometry_msgs::PoseArray
     */
    void set_origin_xyz(double x, double y, double z);

private:
    double a_path_t;
    Eigen::MatrixXd path_points_;
    Eigen::MatrixXd path_points_y_;
    Eigen::MatrixXd path_points_z_;
    Eigen::MatrixXd path_points_3d_;
    Eigen::MatrixXd T_allocation_;
    Eigen::MatrixXd a_;
    Eigen::MatrixXd a_y_;
    Eigen::MatrixXd a_z_;
    Eigen::MatrixXd a_xyz_;

    double origin_x_;
    double origin_y_;
    double origin_z_;

    nav_msgs::Path traj_points_;
    geometry_msgs::PoseArray traj_points_array_;

    // car param
    //double vel_;
    //double acc_;
};

#endif // MINIMUMJERK_H
