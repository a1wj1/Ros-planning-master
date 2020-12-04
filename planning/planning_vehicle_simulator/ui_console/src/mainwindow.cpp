#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ros/ros.h"
#include "ui_console/control.h"
#include <geometry_msgs/Twist.h>
#include "planning_msgs/global_routing.h"



MainWindow::MainWindow(QWidget *parent, int argc, char** argv) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->tabWidget->setTabText(0, QString("Data Display"));
    ui->tabWidget->setTabText(1, QString("Vehicle Control"));
    ui->tabWidget->setTabText(2, QString("Global Routing"));
    ui->groupBox_information->setAlignment(Qt::AlignCenter);

    ui->label_cur_pos_x_val->setNum(double(0.000));
    ui->label_cur_pos_y_val->setNum(double(0.000));
    ui->label_cur_yaw_val->setNum(double(0.000));
    ui->label_cur_speed_val->setNum(double(0.000));

    ros_node_init(argc, argv);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void rec_lanelets_id_call_back(const planning_msgs::lanelets_id& msg)
{
    //std::cout<<"gr_start_id: "<<gr_start_id<<std::endl;
    //std::cout<<"gr_end_id: "<<gr_end_id<<std::endl;
    std::cout<<"lanelets_id receive success success success!!!!! "<<std::endl;
}
void MainWindow::rec_lanelets_id_call_back_(const planning_msgs::lanelets_id& msg)
{
    //std::cout<<"gr_start_id: "<<gr_start_id<<std::endl;
    //std::cout<<"gr_end_id: "<<gr_end_id<<std::endl;
    std::cout<<"lanelets_id receive success success success!!!!! "<<std::endl;
    callback_lanlets_id_ = msg;
}

void MainWindow::ros_node_init(int argc, char** argv)
{
    ros::init(argc, argv, "interface_console");
    ros::NodeHandle n;
    this->global_routing_pub = n.advertise<planning_msgs::global_routing>("global_routing", 10);
    this->control_data_pub = n.advertise<geometry_msgs::Vector3>("control_data", 10);

    this->lanelet_id_msg_sub = n.subscribe("lanelets_id", 1000, MainWindow::rec_lanelets_id_call_back_);
}
void MainWindow::set_display_pos_x(double val)
{
    ui->label_cur_pos_x_val->setNum(val);
}
void MainWindow::set_display_pos_y(double val)
{
    ui->label_cur_pos_y_val->setNum(val);
}
void MainWindow::set_display_yaw(double val)
{
    ui->label_cur_yaw_val->setNum(val);
}
void MainWindow::set_display_speed(double val)
{
    ui->label_cur_speed_val->setNum(val);
}

void MainWindow::on_pushButton_send_clicked()
{
    std::cout<<"publish control data"<<std::endl;
    float speed = ui->lineEdit_send_speed_val->text().toFloat();
    float steer = ui->lineEdit_send_steer_val->text().toFloat();
    //ROS_INFO("SEND DATA");

    geometry_msgs::Vector3 msg;
    msg.x = speed;
    msg.y = steer;
    
    
    if(ros::ok())
    {
        this->control_data_pub.publish(msg);
    
    }
    
}
void MainWindow::on_pushButton_clicked()
{
    ROS_INFO("SEND DATA");
}

planning_msgs::lanelets_id MainWindow::callback_lanlets_id_;

void MainWindow::on_pushButtonSearch_clicked()
{
    planning_msgs::global_routing msg;

    //long int start = ui->lineEditStartPose->text().toLongLong();
    //long int end = ui->lineEditEndPoae->text().toLongLong();
    long int start = ui->comboBox_start_id->currentText().toLong();;
    long int end = ui->comboBox_end_id->currentText().toLong();;

    std::cout<<"start:"<<start<<std::endl;
    std::cout<<"end:"<<end<<std::endl;
    ROS_INFO("SEND DATA");

    msg.start_position_id = start;
    msg.goal_position_id = end;

    if(ros::ok())
    {
        this->global_routing_pub.publish(msg);
    } 
}

void MainWindow::on_pushButtonSearch_update_clicked()
{
    //std::cout<<"----------update---------- "<<std::endl;
    ros::Rate loop_rate(10);

    if(ros::ok())
    {
        std::cout<<"----------update---------- "<<std::endl;
        ros::spinOnce();
        loop_rate.sleep();

        for(int i = 0; i < callback_lanlets_id_.id.size(); i++)
        {
            int64_t numb = callback_lanlets_id_.id[i];
            ui->comboBox_start_id->addItem(QString::number(numb));
            ui->comboBox_end_id->addItem(QString::number(numb));
        }
        //count = 0;
       
    } 
    
}

void MainWindow::on_pushButtonSearch_publish_clicked()
{
    std::cout<<"publish!"<<std::endl;
}

