#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <pcl/visualization/pcl_visualizer.h>

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
    QButtonGroup *mainBtnGrp;

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
        } multimodalDetection;

    }parameter;

public slots:
    void updateTopic();
private slots:
    void getPCDFile();

    void on_Function1_clicked();

    void on_Function2_clicked();

    void on_Function3_clicked();

    void on_Function4_clicked();

private:
    Ui::MainWindow *ui;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    void Hide(QWidget* WidgetToHide[], int size);
};

#endif // MAINWINDOW_H
