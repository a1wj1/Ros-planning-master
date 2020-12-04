#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/Twist.h>

// 接收到订阅的消息后，会进入消息回调函数
void ControlMsgCallback(const geometry_msgs::Vector3 &v)
{
  // 将接收到的消息打印出来
  // ROS_INFO("msg->linear.x: [%f]", v.x);
  // ROS_INFO("msg->linear.y: [%f]", v.y);
  // ROS_INFO("msg->linear.z: [%f]", v.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_publisher");
  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  // 创建一个Subscriber，订阅名为chatter的topic，注册回调函数chatterCallback
  ros::Subscriber sub = n.subscribe("control_data", 1000, ControlMsgCallback);
  //ros::Subscriber sub_waypoint = n.subscribe("waypoints_vis", 1000, ControlMsgCallback);

  ros::Time cur_time, last_time;
  cur_time = ros::Time::now();
  last_time = cur_time;

  // 设置频率
  ros::Rate r(10);

  // param
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  double vx = 0.1;
  double vy = -0.1;
  double vth = 0.1;
  // 主循环
  while (n.ok())
  {
    ros::spinOnce();
    cur_time = ros::Time::now();
    double dt = (cur_time - last_time).toSec();
    double delta_X = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_X;
    y += delta_y;
    th += delta_th;

    // 打印数据

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
    // 发送odom到base_link 坐标系的转换，需要设置header和子坐标系
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = cur_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    // 这里填充里程计的数据
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // 发送
    odom_broadcaster.sendTransform(odom_trans);
    // 填充里程计数据
    nav_msgs::Odometry odom;
    odom.header.stamp = cur_time;
    // 坐标系
    odom.header.frame_id = "odom";
    // 位置信息
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation = odom_quat;
    // 速度信息
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;
    // 发布里程计
    odom_pub.publish(odom);
    last_time = cur_time;

    r.sleep();
  }
  return 0;
}
