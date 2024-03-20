#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <pcl/visualization/pcl_visualizer.h>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"
#include "preprocess/PointCloudWithString.h"
#include "yoloinfer/yoloWithString.h"
#include <QMainWindow>
#include <QButtonGroup>
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void tra_point_cloud_sub_callback(const preprocess::PointCloudWithStringConstPtr& cloud_with_string);
    void mul_point_cloud_sub_callback(const preprocess::PointCloudWithStringConstPtr& cloud_with_string);
    void image_sub_callback(const yoloinfer::yoloWithStringConstPtr& yolo_with_string);
    void lidar_sub_callback(const sensor_msgs::PointCloudConstPtr& lidar_cloud);
    void camera_sub_callback(const sensor_msgs::ImageConstPtr& image);
    QButtonGroup *mainBtnGrp;
    QButtonGroup *traBtnGrp;
    //储存json的结构体
    struct Topicparameter{
        int mode = 0;
        struct {
            int scheme = 1;
            struct {
                double gridSize = 0.1;
            } voxel;
            struct {
                std::string filterAxis = "z";
                double minValue = -10.0;
                double maxValue = -1.2;
                bool inOutRange = true;
            } passThroughGrid;
            struct {
                double distanceThreshold = 0.2;
                int maxIterations = 100;
                double residualRatio = 0.9;
            } planeFitting;
            struct {
                double clusteringRadius = 2.0;
                int minClusterSize = 50;
                int maxClusterSize = 25000;
            } euclideanClustering;
        } traditionalDetection;
        struct {
            int enableImage = 0;
            int enablePointCloud = 0;
            int automaticROI = 0;
        } multimodalDetection;

    }parameter;

public slots:
    void updateTopic();
    void test();
    void callbackSpin();

    void setMode(int mode);
    void setScheme(int scheme);
    void setGridSize(double grid);
    void setFilterAxis(std::string filterAxis);
    void setMinValue(double min);
    void setMaxValue(double max);
    void setInOutRange(bool inOutRange);
    void setDistanceThreshold(double distanceThreshold);
    void setMaxIterationse(int maxIterations);
    void setResidualRatio(double residualRatio);
    void setClusteringRadius(double clusteringRadius);
    void setMinClusterSize(int minClusterSize);
    void setMaxClusterSize(int maxClusterSize);
    void setEnableImage(int enableImage);
    void setEnablePointCloud(int enablePointCloud);
    void setAutomaticROI(int automaticROI);
private slots:
    void resetButton();

    void getPCDFile();

    void on_Function1_clicked();

    void on_Function3_clicked();

    void on_enablePointCloud_toggled(bool checked);

    void on_scheme1_toggled(bool checked);

    void on_scheme2_toggled(bool checked);

    void on_radioButton_z_toggled(bool checked);

    void on_radioButton_y_toggled(bool checked);

    void on_radioButton_x_toggled(bool checked);

    void on_inOutRange_toggled(bool checked);

    void on_enableImage_toggled(bool checked);

    void on_automaticROI_toggled(bool checked);

    void on_shiftButton_toggled(bool checked);

private:
    Ui::MainWindow *ui;

    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;

    void Hide(QWidget* WidgetToHide[], int size);
    
     //连接各个组件的信号和对应的槽，并初始化其他组件的初值
    void connectAssembly();

    QImage convertToQImage(const sensor_msgs::Image& rosImage);
    QImage scaleImage(QImage image);
};

#endif // MAINWINDOW_H
