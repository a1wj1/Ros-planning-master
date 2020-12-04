#include "mainwindow.h"
#include <QApplication>
#include "planning_msgs/global_routing.h"


int main(int argc, char *argv[])
{
    
    planning_msgs::global_routing routing_msg;
    QApplication a(argc, argv);
    MainWindow w(nullptr, argc, argv);
    w.show();

    return a.exec();
}
