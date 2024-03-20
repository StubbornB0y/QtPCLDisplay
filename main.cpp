#include "mainwindow.h"

#include <QApplication>
#include "sensor_msgs/PointCloud.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "preprocess/PointCloudWithString.h"
#include "yoloinfer/yoloWithString.h"
#include "sensor_msgs/Image.h"
//创建句柄，订阅和发布节点

ros::Publisher pubParameter;
ros::Subscriber pre_pcl_sub;
ros::Subscriber yolo_infer_sub;
ros::Subscriber lidar_pcl_sub;
ros::Subscriber camera_sub;
int main(int argc, char *argv[])
{
    ros::init(argc,argv,"publish");
    ros::NodeHandle nh;
    pubParameter = nh.advertise<std_msgs::String>("qt_chatter",5);
    QApplication a(argc, argv);
    MainWindow w;
    yolo_infer_sub = nh.subscribe<yoloinfer::yoloWithString>("yolo_chatter" , 1, &MainWindow::image_sub_callback, &w);
    pre_pcl_sub = nh.subscribe<preprocess::PointCloudWithString>("pre_chatter", 1, &MainWindow::point_cloud_sub_callback, &w);
    lidar_pcl_sub = nh.subscribe<sensor_msgs::PointCloud>("l_chatter",1,&MainWindow::lidar_sub_callback,&w);
    camera_sub = nh.subscribe<sensor_msgs::Image>("c_chatter",1,&MainWindow::camera_sub_callback,&w);
    w.show();
    return a.exec();
}
