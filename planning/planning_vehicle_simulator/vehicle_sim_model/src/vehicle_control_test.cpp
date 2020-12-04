#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <boost/shared_ptr.hpp>

#include "vehicle_model_interface.h"
#include "vehicle_model_ideal.h"
#include "vehicle_model_time_delay.h"
#include "vehicle_model_constant_acceleration.h"

// msg
#include "planning_msgs/global_routing.h"
#include "planning_msgs/lanelets_id.h"
#include "planning_msgs/vehicle_pose.h"

//#include "vehicle_msgs/vehicle_control.h"

#include <geometry_msgs/Twist.h>
std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;

// 接收到订阅的消息后，会进入消息回调函数
/**
 * @brief ControlMsgCallback
 *        接收到订阅的消息后，会进入消息回调函数
 * @param geometry_msgs::Vector3& v  
 */
void ControlMsgCallback(const geometry_msgs::Vector3& v)
{
  // 将接收到的消息打印出来
  // ROS_INFO("msg->linear.x: [%f]", v.x);
  // ROS_INFO("msg->linear.y: [%f]", v.y);
  // ROS_INFO("msg->linear.z: [%f]", v.z);

  float speed= v.x;
  float steer= v.y;

  Eigen::VectorXd input(2);
  input(0) = speed;
  input(1) = steer;
  vehicle_model_ptr_->setInput(input);

}
void VehiclelStartPoseCallback(const planning_msgs::vehicle_pose::ConstPtr& msg)
{
  Eigen::VectorXd state(3);
  state << msg->x, msg->y, msg->yaw;
  vehicle_model_ptr_->setState(state);

 // std::cout<<"VehiclelStartPoseCallback(planning_msgs::vehicle_pose::ConstPtr& msg)"<<state<<std::endl;

}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "odom_publisher");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    // 创建一个Subscriber，订阅名为chatter的topic，注册回调函数chatterCallback
    ros::Subscriber sub = n.subscribe("control_data", 100, ControlMsgCallback);
    ros::Subscriber vehicle_start_pose_sub = n.subscribe("vehicle_start_pose", 10, VehiclelStartPoseCallback);
    //vehicle_msgs::vehicle_control my_msg;
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

    vehicle_model_ptr_ = std::make_shared<VehicleModelIdealSteer>(2.7);
    Eigen::VectorXd state(3);
    state << 0.0, 0.0, 0.0;
    vehicle_model_ptr_->setState(state);

    Eigen::VectorXd input(2);
    input << 0.0, 0.0;
    vehicle_model_ptr_->setInput(input);

    // 主循环
    while(n.ok())
    {
        ros::spinOnce();
        cur_time = ros::Time::now();
        // 
        double dt = (cur_time - last_time).toSec();
        //double delta_X = (vx*cos(th) - vy*sin(th))*dt;
        //double delta_y = (vx*sin(th) + vy*cos(th))*dt;
        //double delta_th = vth * dt;

        //x += delta_X;
        //y += delta_y;
        //th += delta_th;

        vehicle_model_ptr_->update(dt);
        x = vehicle_model_ptr_->getX();
        y = vehicle_model_ptr_->getY();
        th = vehicle_model_ptr_->getYaw();

        // 打印数据
        //ROS_INFO("x: %f", x);
        //ROS_INFO("y: %f", y);
        //ROS_INFO("th: %f", th);

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        // 发送odom到base_link 坐标系的转换，需要设置header和子坐标系
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = cur_time;
        odom_trans.header.frame_id = "base_link";
        odom_trans.child_frame_id = "odom";

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
        odom.header.frame_id = "base_link";
        // 位置信息
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = odom_quat;
        // 速度信息
        odom.child_frame_id = "odom";
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
