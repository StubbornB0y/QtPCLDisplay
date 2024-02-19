#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QLayout>
#include <QDebug>
#include <QButtonGroup>
#include <qpainter.h>
#include "vtkRenderWindow.h"
#include <pcl/common/io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>
#include <qfiledialog.h>
#include <vtkRenderWindow.h>
#include <QVTKRenderWidget.h>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <QVTKOpenGLNativeWidget.h>
#include <QJsonObject>
#include <QJsonDocument>
#include <nlohmann/json.hpp>
#include <QTimer>
using json = nlohmann::json;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("PCLdisplay");

    connectAssembly();

    mainBtnGrp=new QButtonGroup(this);
    mainBtnGrp->setExclusive(true);
    mainBtnGrp->addButton(ui->Function1);
    mainBtnGrp->addButton(ui->Function3);

    traBtnGrp=new QButtonGroup(this);
    traBtnGrp->setExclusive(true);
    traBtnGrp->addButton(ui->scheme1);
    traBtnGrp->addButton(ui->scheme2);

    ui->automaticROI->hide();
    ui->Function1Config->hide();
    ui->Function2Config->hide();
    ui->Function3Config->hide();
    ui->Function4Config->hide();
    layout()->setSizeConstraint(QLayout::SetFixedSize);
    qDebug()<<this->size();
    //pcl::visualization::PCLVisualizer::Ptr viewer;
    cloud=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
    ui->pclwidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->pclwidget->interactor(), ui->pclwidget->renderWindow());
    updateTopic();
    //测试帧数工具，每隔1ms载入点云图像，实测600多帧
    /*
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(test()));// slotCountMessage是我们需要执行的响应函数
    timer->start(1); // 每隔1s
    */

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::Hide(QWidget* WidgetToHide[], int size)
{
    for (int i = 0; i < size; i++)
    {
        WidgetToHide[i]->hide();
    }
}

void MainWindow::getPCDFile()
{
    QString path = QFileDialog::getOpenFileName(0, "Open", "/home/user/Desktop/testpcd", "(*.pcd)");

    if (path.isEmpty() == false)
    {

        if(pcl::io::loadPCDFile(path.toStdString(), *cloud)!=-1)
        {
            ui->PCLwidget_text->hide();
            ui->state_text->setText("Read succeed");
        }
        else
        {
            ui->state_text->setText("Open file Failed!");
        }
        ui->pclwidget->renderWindow()->Render();

        viewer->removeAllPointClouds();
        viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        ui->pclwidget->update();
    }
}

//编辑槽函数：点击时显示对应的bar，再点一下隐藏
void MainWindow::on_Function1_clicked()
{
    QWidget* widgetsToHide[] = { ui->Function2Config, ui->Function3Config, ui->Function4Config };
    int size=sizeof(widgetsToHide) / sizeof(QWidget*);
//    qDebug<<sizeof(widgetsToHide) <<sizeof(QWidget*);
    Hide(widgetsToHide,size);
    if(ui->Function1Config->isVisible())
    {
        ui->Function1Config->hide();
    }
    else{
        ui->Function1Config->show();
    }

}




void MainWindow::on_Function3_clicked()
{
    QWidget* widgetsToHide[] = { ui->Function1Config, ui->Function2Config, ui->Function4Config };
    int size=sizeof(widgetsToHide) / sizeof(QWidget*);
    Hide(widgetsToHide,size);
    ui->scheme1->setChecked(false);
    ui->scheme2->setChecked(false);
    if(ui->Function3Config->isVisible())
    {
        ui->Function3Config->hide();
    }
    else{
        ui->Function3Config->show();
    }
}


void MainWindow::updateTopic()
{
    json j;
    j["mode"] = parameter.mode;
    j["traditionalDetection"]["scheme"] = parameter.traditionalDetection.scheme;
    j["traditionalDetection"]["voxel"]["gridSize"] = parameter.traditionalDetection.voxel.gridSize;
    j["traditionalDetection"]["passThroughGrid"]["filterAxis"] = parameter.traditionalDetection.passThroughGrid.filterAxis;
    j["traditionalDetection"]["passThroughGrid"]["minValue"] = parameter.traditionalDetection.passThroughGrid.minValue;
    j["traditionalDetection"]["passThroughGrid"]["maxValue"] = parameter.traditionalDetection.passThroughGrid.maxValue;
    j["traditionalDetection"]["passThroughGrid"]["inOutRange"] = parameter.traditionalDetection.passThroughGrid.inOutRange;
    j["traditionalDetection"]["planeFitting"]["distanceThreshold"] = parameter.traditionalDetection.planeFitting.distanceThreshold;
    j["traditionalDetection"]["planeFitting"]["maxIterations"] = parameter.traditionalDetection.planeFitting.maxIterations;
    j["traditionalDetection"]["planeFitting"]["residualRatio"] = parameter.traditionalDetection.planeFitting.residualRatio;
    j["traditionalDetection"]["euclideanClustering"]["clusteringRadius"] = parameter.traditionalDetection.euclideanClustering.clusteringRadius;
    j["traditionalDetection"]["euclideanClustering"]["minClusterSize"] = parameter.traditionalDetection.euclideanClustering.minClusterSize;
    j["traditionalDetection"]["euclideanClustering"]["maxClusterSize"] = parameter.traditionalDetection.euclideanClustering.maxClusterSize;
    j["multimodalDetection"]["enableImage"] = parameter.multimodalDetection.enableImage;
    j["multimodalDetection"]["enablePointCloud"] = parameter.multimodalDetection.enablePointCloud;
    j["multimodalDetection"]["automaticROI"] = parameter.multimodalDetection.automaticROI;
    //std::cout << j.dump(4) << std::endl;
    std::string json_str = j.dump();
    std::cout << "Generated JSON string: " << json_str << std::endl;

}

void MainWindow::test(){
    if(pcl::io::loadPCDFile("/home/user/Desktop/testpcd/000038.pcd", *cloud)!=-1)
    {
        ui->PCLwidget_text->hide();
        ui->state_text->setText("Read succeed");
    }
    else
    {
        ui->state_text->setText("Open file Failed!");
    }
    ui->pclwidget->renderWindow()->Render();

    viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    ui->pclwidget->update();
}

void MainWindow::on_enablePointCloud_toggled(bool checked)
{
    qDebug()<<checked;
    if (checked)
    {
        parameter.multimodalDetection.enablePointCloud = 1;
        ui->automaticROI->show();
    }
    else{
        parameter.multimodalDetection.enablePointCloud = 0;
        ui->automaticROI->hide();
    }
    updateTopic();
}


void MainWindow::on_scheme1_toggled(bool checked)
{
    if (checked)
    {
        ui->planeFitting->hide();
        ui->Function2Config->show();
        ui->passThroughGrid->show();

    }
}


void MainWindow::on_scheme2_toggled(bool checked)
{
    if (checked)
    {
        ui->passThroughGrid->hide();
        ui->Function2Config->show();
        ui->planeFitting->show();
    }
}


void MainWindow::connectAssembly()
{
    ui->gridSize->setDouble(true);
    ui->gridSize->setRange(0.01,1.0);
    ui->gridSize->initScrollBar(0.2);
    connect(ui->gridSize, changedSignal_double(double), this, setGridSize(double));

    ui->ptg_minValue->setDouble(true);
    ui->ptg_minValue->setRange(-100.0, -1.0);
    ui->ptg_minValue->initScrollBar(-10.0);
    connect(ui->ptg_minValue, changedSignal_double(double), this, setMinValue(double));
}


void MainWindow::setMode(int mode)
{
    parameter.mode = mode;
}

void MainWindow::setScheme(int scheme)
{
    parameter.traditionalDetection.scheme=scheme;
}

void MainWindow::setGridSize(double grid)
{
    parameter.traditionalDetection.voxel.gridSize = grid;
}

void MainWindow::setFilterAxis(std::string filterAxis)
{
    parameter.traditionalDetection.passThroughGrid.filterAxis = filterAxis;
}

void MainWindow::setMinValue(double min)
{
    parameter.traditionalDetection.passThroughGrid.minValue = min;
}

void MainWindow::setMaxValue(double max)
{
    parameter.traditionalDetection.passThroughGrid.maxValue = max;
}

void MainWindow::setInOutRange(bool inOutRange)
{
    parameter.traditionalDetection.passThroughGrid.inOutRange = inOutRange;
}

void MainWindow::setDistanceThreshold(double distanceThreshold)
{
    parameter.traditionalDetection.planeFitting.distanceThreshold = distanceThreshold;
}

void MainWindow::setMaxIterationse(int maxIterations)
{
    parameter.traditionalDetection.planeFitting.maxIterations = maxIterations；
}

void MainWindow::setResidualRatio(double residualRatio)
{
    parameter.traditionalDetection.planeFitting.residualRatio =residualRatio;
}

void MainWindow::setClusteringRadius(double clusteringRadius)
{
    parameter.traditionalDetection.euclideanClustering.clusteringRadius = clusteringRadius;
}

void MainWindow::setMinClusterSize(int minClusterSize)
{
    parameter.traditionalDetection.euclideanClustering.minClusterSize = minClusterSize;
}

void MainWindow::setMaxClusterSize(int maxClusterSize)
{
    parameter.traditionalDetection.euclideanClustering.maxClusterSize = maxClusterSize;
}

void MainWindow::setEnableImage(int enableImage)
{
    parameter.multimodalDetection.enableImage = enableImage;
}

void MainWindow::setEnablePointCloud(int enablePointCloud)
{
    parameter.multimodalDetection.enablePointCloud = enablePointCloud;
}

void MainWindow::setAutomaticROI(int automaticROI)
{
    parameter.multimodalDetection.automaticROI = automaticROI;
}

