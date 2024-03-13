#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QLayout>
#include <QDebug>
#include <QButtonGroup>
#include <qpainter.h>
#include "vtkRenderWindow.h"

#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/conversions.h>   
#include <pcl_ros/transforms.h>
#include <pcl/common/io.h>
//#include <pcl/io/io.h>
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"

#include "BoundingBox.h"
extern ros::Publisher pubParameter;
using json = nlohmann::json;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("PCLdisplay");

    mainBtnGrp=new QButtonGroup(this);
    mainBtnGrp->setExclusive(true);
    mainBtnGrp->addButton(ui->Function1);
    mainBtnGrp->addButton(ui->Function3);

    traBtnGrp=new QButtonGroup(this);
    traBtnGrp->setExclusive(true);
    traBtnGrp->addButton(ui->scheme1);
    traBtnGrp->addButton(ui->scheme2);
    resetButton();
    connectAssembly();

    layout()->setSizeConstraint(QLayout::SetFixedSize);
    qDebug()<<this->size();
    cloud=pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
    ui->pclwidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->pclwidget->interactor(), ui->pclwidget->renderWindow());
    updateTopic();
    /*
    QImage* ori_image = new QImage();
    ori_image->load("/home/user/Pictures/Predator_Wallpaper_01_3840x2400.jpg");
    QImage image_scaled = scaleImage(*ori_image);
    ui->imageLabel->setPixmap(QPixmap::fromImage(image_scaled));
    */
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(callbackSpin()));// slotCountMessage是我们需要执行的响应函数
    timer->start(10); // 每隔1s
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
            
            BoundingBox boxAnalysis;
            std::vector<BoundingBoxParameter> boxes = boxAnalysis.analysisString(R"({
                "BoundingBox": [
                    {
                        "x": 58.8379,
                        "y": 16.5403,
                        "z": -0.72959,
                        "w": 3.90107,
                        "l": 1.57018,
                        "h": 1.47626,
                        "rt": 3.19253,
                        "id": 0,
                        "score": 0.500488
                    },
                    {
                        "x": 58.8379,
                        "y": 16.5403,
                        "z": -0.72959,
                        "w": 3.90107,
                        "l": 1.57018,
                        "h": 1.47626,
                        "rt": 3.19253,
                        "id": 0,
                        "score": 0.500488
                    },
                    {
                        "x": 58.8379,
                        "y": 16.5403,
                        "z": -0.72959,
                        "w": 3.90107,
                        "l": 1.57018,
                        "h": 1.47626,
                        "rt": 3.19253,
                        "id": 0,
                        "score": 0.500488
                    }
                ]
            })");
            qDebug()<<boxes.size();
            //pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            ui->PCLwidget_text->hide();
            ui->pclwidget->renderWindow()->Render();
            viewer->removeAllPointClouds();
            for(const auto &box:boxes){
                std::string cube_name = "cube_" + std::to_string(box.id);
                viewer->addCube(Eigen::Vector3f(box.x,box.y,box.z),
                                Eigen::Quaternionf(Eigen::AngleAxisf(box.rt, Eigen::Vector3f::UnitZ())),
                                box.w,  //对应weight
                                box.h,  //对应height
                                box.l,  //对应depth
                                cube_name
                                );
                                std::cout << "x: " << box.x << ", y: " << box.y << ", z: " << box.z
                    << ", w: " << box.w << ", l: " << box.l << ", h: " << box.h
                    << ", rt: " << box.rt << ", id: " << box.id << ", score: " << box.score
                    << std::endl;
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube_name);
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, cube_name);
            }     
            /*ui->PCLwidget_text->hide();
            ui->pclwidget->renderWindow()->Render();
            viewer->removeAllPointClouds();*/
            viewer->addPointCloud<pcl::PointXYZI>(cloud, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
            ui->pclwidget->update();
            ui->state_text->setText("Read succeed");
        }
        else
        {
            ui->state_text->setText("Open file Failed!");
        }
    }
}

void MainWindow::point_cloud_sub_callback(const preprocess::PointCloudWithStringConstPtr& cloud_with_string)
{
    sensor_msgs::PointCloud2 cloud = cloud_with_string->point_cloud;
    std_msgs::String bounding_Box = cloud_with_string->custom_string; 
    
    qDebug()<<"I have been called";
    BoundingBox boxAnalysis;
    std::vector<BoundingBoxParameter> boxes = boxAnalysis.analysisString(bounding_Box.data);
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(cloud, *temp_cloud);
    ui->PCLwidget_text->hide();
    ui->pclwidget->renderWindow()->Render();
    viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZI>(temp_cloud, "sample cloud");
    for(const auto &box:boxes){
        std::string cube_name = "cube_" + std::to_string(box.id);
        viewer->addCube(Eigen::Vector3f(box.x,box.y,box.z),
                        Eigen::Quaternionf(Eigen::AngleAxisf(box.rt, Eigen::Vector3f::UnitZ())),
                        box.w,  //对应weight
                        box.h,  //对应height
                        box.l,  //对应depth
                        cube_name
                        );
                        std::cout << "x: " << box.x << ", y: " << box.y << ", z: " << box.z
              << ", w: " << box.w << ", l: " << box.l << ", h: " << box.h
              << ", rt: " << box.rt << ", id: " << box.id << ", score: " << box.score
              << std::endl;
        //设置颜色和去掉表面
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube_name);
	    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, cube_name);
    }    
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    ui->state_text->setText("Received massage");
    ui->pclwidget->update();
}

void MainWindow::image_sub_callback(const yoloinfer::yoloWithStringConstPtr &yolo_with_string)
{
    sensor_msgs::Image image =yolo_with_string->image;
    std_msgs::String bounding_Box = yolo_with_string->custom_string;
    qDebug()<<"Received Image";
    QImage image_from_ros = convertToQImage(image);
    BoundingBox boxAnalysis;
    std::vector<BoundingBoxParameter> boxes = boxAnalysis.analysisString(bounding_Box.data);
    for(const auto &box:boxes){
        QPainter painter(&image_from_ros);

        // 设置绘制参数，如画笔颜色、线宽等
        painter.setPen(Qt::red);
        painter.setBrush(Qt::NoBrush); // 设置填充为无填充，即空心方框
        painter.setRenderHint(QPainter::Antialiasing, true); // 设置抗锯齿

        // 绘制方框
        QRect rect(QPoint(box.x-box.w/2,box.y-box.h/2),QSize(box.w,box.h));
        painter.drawRect(rect);

        // 结束绘制
        painter.end();
    }
    QImage image_scaled = scaleImage(image_from_ros);
    ui->imageLabel->setPixmap(QPixmap::fromImage(image_scaled));
    //ui->->setPixmap(QPixmap::fromImage(*scale_image));
}

//编辑槽函数：点击时显示对应的bar，再点一下隐藏
void MainWindow::on_Function1_clicked()
{
    QWidget* widgetsToHide[] = { ui->Function2Config, ui->multimodal, ui->Function4Config };
    int size=sizeof(widgetsToHide) / sizeof(QWidget*);
//    qDebug<<sizeof(widgetsToHide) <<sizeof(QWidget*);
    Hide(widgetsToHide,size);
    if(ui->Function1Config->isVisible())
    {
        ui->Function1Config->hide();
        qDebug()<<ui->Function1->isChecked();
    }
    else{
        ui->Function1Config->show();
        if(ui->scheme1->isChecked() || ui->scheme2->isChecked()){
            ui->Function2Config->show();
        }
    }
    if(ui->Function1->isChecked()){
        setMode(1);
    }
    else if(ui->Function1->isChecked() == false &&ui->Function3->isChecked() == false){
        setMode(0);
    }
}




void MainWindow::on_Function3_clicked()
{
    QWidget* widgetsToHide[] = { ui->Function1Config, ui->Function2Config, ui->Function4Config };
    int size=sizeof(widgetsToHide) / sizeof(QWidget*);
    Hide(widgetsToHide,size);
    ui->scheme1->setChecked(false);
    ui->scheme2->setChecked(false);
    if(ui->multimodal->isVisible())
    {
        ui->multimodal->hide();

    }
    else{
        ui->multimodal->show();
    }

    if(ui->Function3->isChecked()){
        setMode(2);
    }
    else if(ui->Function1->isChecked() == false && ui->Function3->isChecked() == false){
        setMode(0);
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
    std_msgs::String msg;
    msg.data = json_str;
    pubParameter.publish(msg);
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

void MainWindow::callbackSpin()
{
    ros::spinOnce();
}

void MainWindow::on_enablePointCloud_toggled(bool checked)
{
    qDebug()<<checked;
    if (checked)
    {
        setEnablePointCloud(1);
        ui->automaticROI->show();
    }
    else{
        setEnablePointCloud(0);
        ui->automaticROI->hide();
    }
}


void MainWindow::on_scheme1_toggled(bool checked)
{
    if (checked)
    {
        ui->planeFitting->hide();
        ui->Function2Config->show();
        ui->passThroughGrid->show();
        setScheme(1);
    }
}


void MainWindow::on_scheme2_toggled(bool checked)
{
    if (checked)
    {
        ui->passThroughGrid->hide();
        ui->Function2Config->show();
        ui->planeFitting->show();
        setScheme(2);
    }
}

void MainWindow::resetButton()
{
    ui->automaticROI->hide();
    ui->Function1Config->hide();
    ui->Function2Config->hide();
    ui->multimodal->hide();
    ui->Function4Config->hide();

    parameter.mode = 0;
    
    // 重置 traditionalDetection 成员变量
    parameter.traditionalDetection.scheme = 1;
    parameter.traditionalDetection.voxel.gridSize = 0.1;
    parameter.traditionalDetection.passThroughGrid.filterAxis = "z";
    parameter.traditionalDetection.passThroughGrid.minValue = -10.0;
    parameter.traditionalDetection.passThroughGrid.maxValue = -1.2;
    parameter.traditionalDetection.passThroughGrid.inOutRange = true;
    parameter.traditionalDetection.planeFitting.distanceThreshold = 0.2;
    parameter.traditionalDetection.planeFitting.maxIterations = 100;
    parameter.traditionalDetection.planeFitting.residualRatio = 0.9;
    parameter.traditionalDetection.euclideanClustering.clusteringRadius = 2.0;
    parameter.traditionalDetection.euclideanClustering.minClusterSize = 50;
    parameter.traditionalDetection.euclideanClustering.maxClusterSize = 25000;

    // 重置 multimodalDetection 成员变量
    parameter.multimodalDetection.enableImage = 0;
    parameter.multimodalDetection.enablePointCloud = 0;
    parameter.multimodalDetection.automaticROI = 0;

    //mode
    ui->Function1->setChecked(false);
    ui->Function3->setChecked(false);

    //voxel
    ui->gridSize->setDouble(true);
    ui->gridSize->setRange(0.01,100.0);
    ui->gridSize->initScrollBar(0.1);
    ui->gridSize->lineTextEdit->setText(QString::number(0.1,'f',2));

    //passThroughGrid
    ui->radioButton_z->setChecked(true);
    ui->radioButton_y->setChecked(false);
    ui->radioButton_x->setChecked(false);

    ui->ptg_minValue->setDouble(true);
    ui->ptg_minValue->setRange(-100.0, -1.0);
    ui->ptg_minValue->initScrollBar(-10.0);
    ui->ptg_minValue->lineTextEdit->setText(QString::number(-10.0,'f',2));

    ui->ptg_maxValue->setDouble(true);
    ui->ptg_maxValue->setRange(-10.0, 100.0);
    ui->ptg_maxValue->initScrollBar(-1.2);
    ui->ptg_maxValue->lineTextEdit->setText(QString::number(-1.2,'f',2));

    ui->inOutRange->setChecked(true);
    //planeFitting
    ui->distanceThreshold->setDouble(true);
    ui->distanceThreshold->setRange(0.01,10.0);
    ui->distanceThreshold->initScrollBar(0.2);
    ui->distanceThreshold->lineTextEdit->setText(QString::number(0.2,'f',2));

    ui->maxIterations->setDouble(false);
    ui->maxIterations->setRange(10,10000);
    ui->maxIterations->initScrollBar(100);
    ui->maxIterations->lineTextEdit->setText(QString::number(100));

    ui->residualRatio->setDouble(true);
    ui->residualRatio->setRange(0.01,1.0);
    ui->residualRatio->initScrollBar(0.9);
    ui->residualRatio->lineTextEdit->setText(QString::number(0.9,'f',2));

    //euclidean
    ui->clusteringRadius->setDouble(true);
    ui->clusteringRadius->setRange(0.1,100.0);
    ui->clusteringRadius->initScrollBar(2.0);
    ui->clusteringRadius->lineTextEdit->setText(QString::number(2.0,'f',2));

    ui->minClusterSize->setDouble(false);
    ui->minClusterSize->setRange(10,1000);
    ui->minClusterSize->initScrollBar(50);
    ui->minClusterSize->lineTextEdit->setText(QString::number(50));

    ui->maxClusterSize->setDouble(false);
    ui->maxClusterSize->setRange(500,100000);
    ui->maxClusterSize->initScrollBar(25000);
    ui->maxClusterSize->lineTextEdit->setText(QString::number(25000));

    //multimodalDetection
    ui->enableImage->setChecked(false);
    ui->enablePointCloud->setChecked(false);
    ui->automaticROI->setChecked(false);
}

void MainWindow::connectAssembly()
{
    connect(ui->shiftButton, SIGNAL(toggled(bool)), this, SLOT(on_shiftButton_toggled(bool)));

    connect(ui->reset, SIGNAL(clicked()), this, SLOT(resetButton()));

    connect(ui->gridSize, SIGNAL(changedSignal_double(double)), this, SLOT(setGridSize(double)));

    connect(ui->ptg_minValue, SIGNAL(changedSignal_double(double)), this, SLOT(setMinValue(double)));
    connect(ui->ptg_maxValue, SIGNAL(changedSignal_double(double)), this, SLOT(setMaxValue(double)));

    connect(ui->distanceThreshold, SIGNAL(changedSignal_double(double)), this, SLOT(setDistanceThreshold(double)));
    connect(ui->maxIterations, SIGNAL(changedSignal_int(int)), this, SLOT(setMaxIterationse(int)));
    connect(ui->residualRatio, SIGNAL(changedSignal_double(double)), this, SLOT(setResidualRatio(double)));

    connect(ui->clusteringRadius, SIGNAL(changedSignal_double(double)), this, SLOT(setClusteringRadius(double)));
    connect(ui->minClusterSize, SIGNAL(changedSignal_int(int)), this, SLOT(setMinClusterSize(int)));
    connect(ui->maxClusterSize, SIGNAL(changedSignal_int(int)), this, SLOT(setMaxClusterSize(int)));

}

void MainWindow::setMode(int mode)
{
    parameter.mode = mode;
    updateTopic();
}

void MainWindow::setScheme(int scheme)
{
    parameter.traditionalDetection.scheme=scheme;
    updateTopic();
}

void MainWindow::setGridSize(double grid)
{
    parameter.traditionalDetection.voxel.gridSize = grid;
    updateTopic();
}

void MainWindow::setFilterAxis(std::string filterAxis)
{
    parameter.traditionalDetection.passThroughGrid.filterAxis = filterAxis;
    updateTopic();
}

void MainWindow::setMinValue(double min)
{
    parameter.traditionalDetection.passThroughGrid.minValue = min;
    updateTopic();
}

void MainWindow::setMaxValue(double max)
{
    parameter.traditionalDetection.passThroughGrid.maxValue = max;
    updateTopic();
}

void MainWindow::setInOutRange(bool inOutRange)
{
    parameter.traditionalDetection.passThroughGrid.inOutRange = inOutRange;
    updateTopic();
}

void MainWindow::setDistanceThreshold(double distanceThreshold)
{
    parameter.traditionalDetection.planeFitting.distanceThreshold = distanceThreshold;
    updateTopic();
}

void MainWindow::setMaxIterationse(int maxIterations)
{
    parameter.traditionalDetection.planeFitting.maxIterations = maxIterations;
    updateTopic();
}

void MainWindow::setResidualRatio(double residualRatio)
{
    parameter.traditionalDetection.planeFitting.residualRatio =residualRatio;
    updateTopic();
}

void MainWindow::setClusteringRadius(double clusteringRadius)
{
    parameter.traditionalDetection.euclideanClustering.clusteringRadius = clusteringRadius;
    updateTopic();
}

void MainWindow::setMinClusterSize(int minClusterSize)
{
    parameter.traditionalDetection.euclideanClustering.minClusterSize = minClusterSize;
    updateTopic();
}

void MainWindow::setMaxClusterSize(int maxClusterSize)
{
    parameter.traditionalDetection.euclideanClustering.maxClusterSize = maxClusterSize;
    updateTopic();
}

void MainWindow::setEnableImage(int enableImage)
{
    parameter.multimodalDetection.enableImage = enableImage;
    updateTopic();
}

void MainWindow::setEnablePointCloud(int enablePointCloud)
{
    parameter.multimodalDetection.enablePointCloud = enablePointCloud;
    updateTopic();
}

void MainWindow::setAutomaticROI(int automaticROI)
{
    parameter.multimodalDetection.automaticROI = automaticROI;
    updateTopic();
}


void MainWindow::on_radioButton_z_toggled(bool checked)
{
    if(checked){
        setFilterAxis("z");
    }
}


void MainWindow::on_radioButton_y_toggled(bool checked)
{
    if(checked){
        setFilterAxis("y");
    }
}


void MainWindow::on_radioButton_x_toggled(bool checked)
{
    if(checked){
        setFilterAxis("x");
    }
}

void MainWindow::on_inOutRange_toggled(bool checked)
{
    if(checked){
        setInOutRange(true);
    }
    else{
        setInOutRange(false);
    }
}


void MainWindow::on_enableImage_toggled(bool checked)
{
    if(checked){
        setEnableImage(1);
    }
    else{
        setEnableImage(0);
    }
}


void MainWindow::on_automaticROI_toggled(bool checked)
{
    if(checked){
        setAutomaticROI(1);
    }
    else{
        setAutomaticROI(0);
    }
}

void MainWindow::on_shiftButton_toggled(bool checked)
{
    if(checked){
        ui->imageLabel->setMinimumSize(800,550);
        ui->imageLabel->setParent(ui->PCLViewBox);
        ui->imageLabel->setGeometry(12,32,800,550);
        ui->pclwidget->setParent(ui->imageLabel);
        ui->pclwidget->setMinimumSize(331,201);
        ui->pclwidget->setGeometry(0,0,331,201);
    }
    else{
        ui->pclwidget->setMinimumSize(800,550);
        ui->pclwidget->setParent(ui->PCLViewBox);
        ui->pclwidget->setGeometry(12,32,800,550);
        ui->imageLabel->setParent(ui->pclwidget);
        ui->imageLabel->setMinimumSize(331,201);
        ui->imageLabel->setGeometry(0,0,331,201);
    }
}

QImage MainWindow::convertToQImage(const sensor_msgs::Image& rosImage)
{
    QImage::Format format;

    // 根据ROS图像的编码格式设置QImage的格式
    if (rosImage.encoding == "rgb8")
    {
        format = QImage::Format_RGB888;
    }
    if (rosImage.encoding == "bgr8")
    {
        format = QImage::Format_BGR888;
    }
    else if (rosImage.encoding == "rgba8" || rosImage.encoding == "bgra8")
    {
        format = QImage::Format_RGBA8888;
    }
    else if (rosImage.encoding == "mono8")
    {
        format = QImage::Format_Grayscale8;
    }
    else
    {
        // 不支持的编码格式
        return QImage();
    }

    // 创建QImage对象
    QImage qImage(rosImage.data.data(), rosImage.width, rosImage.height, format);
    //如果是BGR格式则转换
    if(format == QImage::Format_BGR888){
        qImage=qImage.rgbSwapped();
    }
    // 如果ROS图像的步长与每行的字节数不匹配，则进行复制
    if (rosImage.step != qImage.bytesPerLine())
    {
        QImage copiedImage = qImage.copy();
        return copiedImage;
    }
    return qImage;
}

QImage MainWindow::scaleImage(QImage image)
{
    int ori_width = image.size().width();
    int ori_height = image.size().height();

    int m_width = ui->imageLabel->size().width();
    int m_height = ui->imageLabel->size().height();

    int pro;
    if(ori_width / m_width >= ori_height / m_height)
        pro = ori_width / m_width;
    else
        pro = ori_height / m_height;

    int scale_width = ori_width / pro;
    int scale_height = ori_height / pro;
    QImage scale_image = image.scaled(scale_width, scale_height, Qt::KeepAspectRatio);
    return scale_image;
}
