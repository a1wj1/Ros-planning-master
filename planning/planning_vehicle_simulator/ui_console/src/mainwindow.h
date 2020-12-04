#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>

// ros msg
#include "planning_msgs/lanelets_id.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent , int argc, char** argv);
    ~MainWindow();

private slots:


    void on_pushButton_send_clicked();
    void on_pushButton_clicked();
    void on_pushButtonSearch_clicked();
    void on_pushButtonSearch_publish_clicked();

    // ros callback function
    static void rec_lanelets_id_call_back_(const planning_msgs::lanelets_id& msg);
    void on_pushButtonSearch_update_clicked();

private:
    Ui::MainWindow *ui;

    void ros_node_init(int argc, char** argv);

    void set_display_pos_x(double val);
    void set_display_pos_y(double val);
    void set_display_yaw(double val);
    void set_display_speed(double val);

    // ros
    ros::Publisher control_data_pub;
    ros::Publisher global_routing_pub;
    ros::Subscriber lanelet_id_msg_sub;

    static planning_msgs::lanelets_id callback_lanlets_id_;


};


#endif // MAINWINDOW_H
