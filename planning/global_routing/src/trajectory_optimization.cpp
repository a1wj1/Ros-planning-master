#include "trajectory_optimization.h"
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Path.h>
TrajectoryOptimization::TrajectoryOptimization()
{
  std::cout << "TrajectoryOptimization" << std::endl;
}
/*
由于lanelet的搜索是基于车道，起点和终点都会以车段的id去划分，
为了实现点到点的全剧路径规划，需要进行进一步切割，
把车段的不用显示和跑的路径切割掉
*/
void TrajectoryOptimization::trajectory_start_cutting(geometry_msgs::PoseArray &path_points, geometry_msgs::PoseArray &path_after_cutting, int index, Point3d_s &target_pose, Point3d_s &set_pose)
{
  //参数说明：path_points：输入的车段id的路径点信息。path_after_cutting：输出的切割后的路径点信息。index：输入的里起点或终点最近的点的id。
  //target_pose:输入的里起点或终点最近的点。set_pose：输入的起点，因为是一样的优化算法

  //（1）第一种情况： 最近的点p2不是第一个点和最后一个点的时候,需要考虑前后两个点，总共连着三个点，设为p1,p2,p3。也就是判断起点的在哪两个点之间
  if (index != 0 && index != (path_points.poses.size() - 1))
  {
    //获取前后两个点，总共连着三个点,并且求出两点之间的斜率
    Point2d_s prevent_pp;
    Point2d_s after_pp;
    prevent_pp.x = path_points.poses[index - 1].position.x; //p1
    prevent_pp.y = path_points.poses[index - 1].position.y;
    after_pp.x = path_points.poses[index + 1].position.x; //p3
    after_pp.y = path_points.poses[index + 1].position.y;
    double k1, k2;
    k1 = (prevent_pp.y - target_pose.y) / (prevent_pp.x - target_pose.x);
    k2 = (after_pp.y - target_pose.y) / (after_pp.x - target_pose.x);
    //求两点之间的中点坐标,有两个中点坐标
    Point2d_s center_p1;
    Point2d_s center_p2;
    center_p1.x = (target_pose.x + prevent_pp.x) / 2;
    center_p1.y = (target_pose.y + prevent_pp.y) / 2;
    center_p2.x = (target_pose.x + after_pp.x) / 2;
    center_p2.y = (target_pose.y + after_pp.y) / 2;
    //分别求中点坐标到最近点的距离
    double distance1 = sqrt((center_p1.x - target_pose.x) * (center_p1.x - target_pose.x) + (center_p1.y - target_pose.y) * (center_p1.y - target_pose.y));
    double distance2 = sqrt((center_p2.x - target_pose.x) * (center_p2.x - target_pose.x) + (center_p2.y - target_pose.y) * (center_p2.y - target_pose.y));
    //求中垂线的方程,这种情况有两条
    double x1, y1, x2, y2;
    y1 = (-1 / k1) * (x1 - center_p1.x) + center_p1.y;
    y2 = (-1 / k2) * (x2 - center_p2.x) + center_p2.y;
    //求起点到直线的距离
    //double d1 = fabs(A * px + B * py + C1) / sqrt(pow(A, 2) + pow(B, 2));
    double d1 = fabs(1 * set_pose.x + k1 * set_pose.y - center_p1.x - k1 * center_p1.y) / sqrt(pow(1, 2) + pow(k1, 2));
    double d2 = fabs(1 * set_pose.x + k2 * set_pose.y - center_p2.x - k2 * center_p2.y) / sqrt(pow(1, 2) + pow(k2, 2));
    //比较距离大小，从而判断终点的位置
    if ((d1 <= distance1) && (d2 >= distance2)) //在p1和p2之间，把p1之前的删除，在p2前面插入起点
    {
      point_posess.position.x = set_pose.x;
      point_posess.position.y = set_pose.y;
      point_posess.position.z = set_pose.z;
      path_after_cutting.poses.push_back(point_posess);
      for (int i = index; i < path_points.poses.size(); i++)
      {
        point_posess.position.x = path_points.poses[i].position.x;
        point_posess.position.y = path_points.poses[i].position.y;
        point_posess.position.z = path_points.poses[i].position.z;
        path_after_cutting.poses.push_back(point_posess);
      }
    }
    else if (d1 > distance1 && d2 < distance2) //在p2和p3之间，把p3之前的删除，在p3前面插入起点
    {
      point_posess.position.x = set_pose.x;
      point_posess.position.y = set_pose.y;
      point_posess.position.z = set_pose.z;
      path_after_cutting.poses.push_back(point_posess);
      for (int i = index + 1; i < path_points.poses.size(); i++)
      {
        point_posess.position.x = path_points.poses[i].position.x;
        point_posess.position.y = path_points.poses[i].position.y;
        point_posess.position.z = path_points.poses[i].position.z;
        path_after_cutting.poses.push_back(point_posess);
      }
    }
    // std::cout << "START1" << std::endl;

    // std::cout << "    set_pose:"
    //           << "(" << set_pose.x << "," << set_pose.y << ")" << std::endl;
    // std::cout << "target_pose:"
    //           << "(" << target_pose.x << "," << target_pose.y << ")" << std::endl;
    // std::cout << "prevent_pp:"
    //           << "(" << prevent_pp.x << "," << prevent_pp.y << ")" << std::endl;
    // std::cout << "after_pp:"
    //           << "(" << after_pp.x << "," << after_pp.y << ")" << std::endl;
    // std::cout << " d1:" << d1 << std::endl;
    // std::cout << " distance1:" << distance1 << std::endl;
    // std::cout << " d2:" << d2 << std::endl;
    // std::cout << " distance2:" << distance2 << std::endl;
    // std::cout << "pathsize():" << path_after_cutting.poses.size() << std::endl;
  }
  //（2）第二种情况： 最近的点是第一个点而不是最后一个点的时候,起点切后面，替换第一个点
  else if (index == 0 && index != (path_points.poses.size() - 1))
  {
    point_posess.position.x = set_pose.x;
    point_posess.position.y = set_pose.y;
    point_posess.position.z = set_pose.z;
    path_after_cutting.poses.push_back(point_posess);
    for (int i = index + 1; i < path_points.poses.size(); i++)
    {
      point_posess.position.x = path_points.poses[i].position.x;
      point_posess.position.y = path_points.poses[i].position.y;
      point_posess.position.z = path_points.poses[i].position.z;
      path_after_cutting.poses.push_back(point_posess);
    }
    // std::cout << "START2" << std::endl;
  }
  //（3）第三种情况： 最近的点不是第一个点而是最后一个点的时候,起点切后面，替换最后一个点
  else if (index != 0 && index == (path_points.poses.size() - 1))
  {
    point_posess.position.x = set_pose.x;
    point_posess.position.y = set_pose.y;
    point_posess.position.z = set_pose.z;
    path_after_cutting.poses.push_back(point_posess);
    point_posess.position.x = path_points.poses[path_points.poses.size() - 1].position.x;
    point_posess.position.y = path_points.poses[path_points.poses.size() - 1].position.y;
    point_posess.position.z = path_points.poses[path_points.poses.size() - 1].position.z;
    path_after_cutting.poses.push_back(point_posess);
    // std::cout << "START3" << std::endl;
  }
  //（4）第4种情况： 最近的点既是第一个点也是最后一个点的时候,也就是车段只有一个点，少出现
  else if (index == 0 && index == (path_points.poses.size() - 1))
  {
    point_posess.position.x = set_pose.x;
    point_posess.position.y = set_pose.y;
    point_posess.position.z = set_pose.z;
    path_after_cutting.poses.push_back(point_posess);
    point_posess.position.x = path_points.poses[path_points.poses.size() - 1].position.x;
    point_posess.position.y = path_points.poses[path_points.poses.size() - 1].position.y;
    point_posess.position.z = path_points.poses[path_points.poses.size() - 1].position.z;
    path_after_cutting.poses.push_back(point_posess);
    // std::cout << "START4" << std::endl;
  }
}

void TrajectoryOptimization::trajectory_stop_cutting(geometry_msgs::PoseArray &path_pointss, geometry_msgs::PoseArray &path_after_cuttings, int indexs, Point3d_s &target_poses, Point3d_s &set_poses)
{
  //参数说明：path_points：输入的车段id的路径点信息。path_after_cutting：输出的切割后的路径点信息。index：输入的里起点或终点最近的点的id。
  //target_pose:输入的里起点或终点最近的点。set_pose：输入的起点或者终点，因为是一样的优化算法
  geometry_msgs::Pose point_posess;
  //（1）第一种情况： 最近的点p2不是第一个点和最后一个点的时候,需要考虑前后两个点，总共连着三个点，设为p1,p2,p3。也就是判断终点的在哪两个点之间
  if (indexs != 0 && indexs != (path_pointss.poses.size() - 1))
  {
    //获取前后两个点，总共连着三个点,并且求出两点之间的斜率
    Point2d_s prevent_pp;
    Point2d_s after_pp;
    prevent_pp.x = path_pointss.poses[indexs - 1].position.x; //p1
    prevent_pp.y = path_pointss.poses[indexs - 1].position.y;
    after_pp.x = path_pointss.poses[indexs + 1].position.x; //p3
    after_pp.y = path_pointss.poses[indexs + 1].position.y;
    double k1, k2;
    k1 = (prevent_pp.y - target_poses.y) / (prevent_pp.x - target_poses.x);
    k2 = (after_pp.y - target_poses.y) / (after_pp.x - target_poses.x);
    //求两点之间的中点坐标,有两个中点坐标
    Point2d_s center_p1;
    Point2d_s center_p2;
    center_p1.x = (target_poses.x + prevent_pp.x) / 2;
    center_p1.y = (target_poses.y + prevent_pp.y) / 2;
    center_p2.x = (target_poses.x + after_pp.x) / 2;
    center_p2.y = (target_poses.y + after_pp.y) / 2;
    //分别求中点坐标到最近点的距离
    double distance1 = sqrt((center_p1.x - target_poses.x) * (center_p1.x - target_poses.x) + (center_p1.y - target_poses.y) * (center_p1.y - target_poses.y));
    double distance2 = sqrt((center_p2.x - target_poses.x) * (center_p2.x - target_poses.x) + (center_p2.y - target_poses.y) * (center_p2.y - target_poses.y));
    //求中垂线的方程,这种情况有两条
    double x1, y1, x2, y2;
    y1 = (-1 / k1) * (x1 - center_p1.x) + center_p1.y;
    y2 = (-1 / k2) * (x2 - center_p2.x) + center_p2.y;
    //求终点到直线的距离
    //double d1 = fabs(A * px + B * py + C1) / sqrt(pow(A, 2) + pow(B, 2));
    double d1 = fabs(1 * set_poses.x + k1 * set_poses.y - center_p1.x - k1 * center_p1.y) / sqrt(pow(1, 2) + pow(k1, 2));
    double d2 = fabs(1 * set_poses.x + k2 * set_poses.y - center_p2.x - k2 * center_p2.y) / sqrt(pow(1, 2) + pow(k2, 2));
    //比较距离大小，从而判断终点的位置
    if ((d1 <= distance1) && (d2 >= distance2)) //在p1和p2之间，把p1之后的删除，在p1后面插入终点
    {
      for (int i = 0; i < indexs; i++)
      {
        point_posess.position.x = path_pointss.poses[i].position.x;
        point_posess.position.y = path_pointss.poses[i].position.y;
        point_posess.position.z = path_pointss.poses[i].position.z;
        path_after_cuttings.poses.push_back(point_posess);
      }
      point_posess.position.x = set_poses.x;
      point_posess.position.y = set_poses.y;
      point_posess.position.z = set_poses.z;
      path_after_cuttings.poses.push_back(point_posess);
    }
    else if (d1 > distance1 && d2 < distance2) //在p2和p3之间，把p3之后的删除，在p3前面插入终点
    {
      for (int i = 0; i < indexs + 1; i++)
      {
        point_posess.position.x = path_pointss.poses[i].position.x;
        point_posess.position.y = path_pointss.poses[i].position.y;
        point_posess.position.z = path_pointss.poses[i].position.z;
        path_after_cuttings.poses.push_back(point_posess);
      }
      point_posess.position.x = set_poses.x;
      point_posess.position.y = set_poses.y;
      point_posess.position.z = set_poses.z;
      path_after_cuttings.poses.push_back(point_posess);
    }
    // std::cout << "STOP1" << std::endl;
  }
  //（2）第二种情况： 最近的点是第一个点而不是最后一个点的时候,起点切后面，替换第一个点
  else if (indexs == 0 && indexs != (path_pointss.poses.size() - 1))
  {
    point_posess.position.x = path_pointss.poses[0].position.x;
    point_posess.position.y = path_pointss.poses[0].position.y;
    point_posess.position.z = path_pointss.poses[0].position.z;
    path_after_cuttings.poses.push_back(point_posess);
    point_posess.position.x = set_poses.x;
    point_posess.position.y = set_poses.y;
    point_posess.position.z = set_poses.z;
    path_after_cuttings.poses.push_back(point_posess);
    // std::cout << "STOP2" << std::endl;
  }
  //（3）第三种情况： 最近的点不是第一个点而是最后一个点的时候,起点切后面，替换最后一个点
  else if (indexs != 0 && indexs == (path_pointss.poses.size() - 1))
  {
    for (int i = 0; i < path_pointss.poses.size() - 1; i++)
    {
      point_posess.position.x = path_pointss.poses[i].position.x;
      point_posess.position.y = path_pointss.poses[i].position.y;
      point_posess.position.z = path_pointss.poses[i].position.z;
      path_after_cuttings.poses.push_back(point_posess);
    }
    point_posess.position.x = set_poses.x;
    point_posess.position.y = set_poses.y;
    point_posess.position.z = set_poses.z;
    path_after_cuttings.poses.push_back(point_posess);
    // std::cout << "STOP3" << std::endl;
  }
  //（4）第4种情况： 最近的点既是第一个点也是最后一个点的时候,也就是车段只有一个点，少出现
  else if (indexs == 0 && indexs == (path_pointss.poses.size() - 1))
  {
    point_posess.position.x = set_poses.x;
    point_posess.position.y = set_poses.y;
    point_posess.position.z = set_poses.z;
    path_after_cuttings.poses.push_back(point_posess);
    point_posess.position.x = path_pointss.poses[path_pointss.poses.size() - 1].position.x;
    point_posess.position.y = path_pointss.poses[path_pointss.poses.size() - 1].position.y;
    point_posess.position.z = path_pointss.poses[path_pointss.poses.size() - 1].position.z;
    path_after_cuttings.poses.push_back(point_posess);
    // std::cout << "STOP4" << std::endl;
  }
}

void TrajectoryOptimization::trajectory_same_cutting(geometry_msgs::PoseArray &path_points, geometry_msgs::PoseArray &start_path_points, geometry_msgs::PoseArray &stop_path_points, geometry_msgs::PoseArray &path_after_cutting, int index, Point3d_s &target_pose, Point3d_s &set_pose, int indexs, Point3d_s &target_poses, Point3d_s &set_poses)
{
  //遍历获取重叠的部分
  //放起点
  point_posess.position.x = set_pose.x;
  point_posess.position.y = set_pose.y;
  point_posess.position.z = set_pose.z;
  path_after_cutting.poses.push_back(point_posess);
  //放重复的点
  for (int i = 1; i < start_path_points.poses.size(); i++) //从起点后一个点开始,第一个点是起点 i=0
  {
    for (int j = 0; j < stop_path_points.poses.size() - 1; j++) //从第一个点开始到倒数第二个点,最后一个点是终点 j=stop_path_points.poses.size() - 1
    {
      if ((start_path_points.poses[i].position.x == stop_path_points.poses[j].position.x) && (start_path_points.poses[i].position.y == stop_path_points.poses[j].position.y))
      {
        point_posess.position.x = start_path_points.poses[i].position.x;
        point_posess.position.y = start_path_points.poses[i].position.y;
        point_posess.position.z = start_path_points.poses[i].position.z;
        path_after_cutting.poses.push_back(point_posess);
        // std::cout << "efficient" << std::endl;
      }
    }
  }
  //放终点
  point_posess.position.x = set_poses.x;
  point_posess.position.y = set_poses.y;
  point_posess.position.z = set_poses.z;
  path_after_cutting.poses.push_back(point_posess);
  // std::cout << " size():" << path_after_cutting.poses.size() << std::endl;
}

//正在使用：梯形加速计算时间分配,轨迹优化
/*
时间坐标采用相对时间，每段的时间相同，在C++下采用的是一种以最大加速度和减速度完成每一段路径的轨迹运行。
每段开始时以最大加速度进行加速，当到达最大速度是，采用匀速运行，
当每段快结束时，采用最大减速度进行减速
*/
bool TrajectoryOptimization::acc_time_allocation(Eigen::MatrixXd &path_points, double vel, double acc, Eigen::MatrixXd &time_allo)
{
  time_allo = Eigen::MatrixXd::Zero(path_points.rows() - 1, 1);
  for (int i = 0; i < time_allo.rows(); i++)
  {
    double dis = (path_points.row(i + 1) - path_points.row(i)).norm();
    double t1 = vel / acc;
    double x1 = (vel * vel) / (2 * acc);
    double x2 = dis - 2 * x1;
    double t2 = x2 / vel;
    time_allo(i, 0) = 2 * t1 + t2;
  }
  return true;
}

//正在使用：加权平均
void TrajectoryOptimization::average_interpolation(Eigen::MatrixXd &input, Eigen::MatrixXd &output, double interval_dis, double distance)
{
  //1.定义一个容器，类型为Point3d_s,即（x,y,z）
  std::vector<Point3d_s> vec_3d;
  std::vector<Point3d_s> n_vec;
  Point3d_s p;
  //2.遍历
  // std::cout << " input.rows()" << input.rows() << std::endl;
  for (int i = 0; i < input.rows() - 1; i++)
  {
    double dis = (input.row(i + 1) - input.row(i)).norm(); //求两点的距离，前一行和这一行坐标的距离
    std::cout << "dis " << dis << std::endl;
    //两点距离太长的话就进行插点
    if (dis > distance)
    {
      //计算(x,y)两点的距离
      double sqrt_val = sqrt((input(i + 1, 0) - input(i, 0)) * (input(i + 1, 0) - input(i, 0)) + (input(i + 1, 1) - input(i, 1)) * (input(i + 1, 1) - input(i, 1)));
      //计算角度
      double sin_a = (input(i + 1, 1) - input(i, 1)) / sqrt_val;
      double cos_a = (input(i + 1, 0) - input(i, 0)) / sqrt_val;
      //两点之间要插值的插值点的数量
      int num = dis / interval_dis; //分割了一下
      // std::cout << "num " << num << std::endl;
      //插入点
      for (int j = 0; j < num; j++)
      {
        //i=0,j=0的时候其实是插入起点
        p.x = input(i, 0) + j * interval_dis * cos_a;
        p.y = input(i, 1) + j * interval_dis * sin_a;
        p.z = input(i, 2);
        vec_3d.push_back(p);
      }
    }
    //3.有些点原本比较近，不需要插点，但是也要补进去，不然会缺失,dis >= 1防止点太密集
    else if (dis <= distance && dis >= 1)
    {
      p.x = input(i, 0);
      p.y = input(i, 1);
      p.z = input(i, 2);
      vec_3d.push_back(p);
    }
  }
  //4.漏了终点，需要加上
  p.x = input(input.rows() - 1, 0);
  p.y = input(input.rows() - 1, 1);
  p.z = input(input.rows() - 1, 2);
  vec_3d.push_back(p);

  //传给输出矩阵output
  output = Eigen::MatrixXd::Zero(vec_3d.size(), 3);
  int j = 0;
  for (std::vector<Point3d_s>::iterator it = vec_3d.begin(); it != vec_3d.end(); it++)
  {
    output(j, 0) = (*it).x;
    output(j, 1) = (*it).y;
    output(j, 2) = (*it).z;
    j++;
  }
}