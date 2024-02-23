#include "mainwindow.h"

#include <QApplication>
#include "sensor_msgs/PointCloud2.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
//创建句柄，订阅和发布节点

ros::Publisher pubParameter;
ros::Subscriber point_cloud_sub;
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"publish");
    ros::NodeHandle nh;
    pubParameter = nh.advertise<std_msgs::String>("qt_chatter",5);
    QApplication a(argc, argv);
    MainWindow w;
    
    point_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("pre_chatter", 1, &MainWindow::point_cloud_sub_callback, &w);
    w.show();
    return a.exec();
}
