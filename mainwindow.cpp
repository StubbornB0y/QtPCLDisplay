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


void MainWindow::on_Function2_clicked()
{
    QWidget* widgetsToHide[] = { ui->Function1Config, ui->Function3Config, ui->Function4Config };
    int size=sizeof(widgetsToHide) / sizeof(QWidget*);
    Hide(widgetsToHide,size);
    if(ui->Function2Config->isVisible())
    {
        ui->Function2Config->hide();
    }
    else{
        ui->Function2Config->show();
    }

}


void MainWindow::on_Function3_clicked()
{
    QWidget* widgetsToHide[] = { ui->Function1Config, ui->Function2Config, ui->Function4Config };
    int size=sizeof(widgetsToHide) / sizeof(QWidget*);
    Hide(widgetsToHide,size);
    if(ui->Function3Config->isVisible())
    {
        ui->Function3Config->hide();
    }
    else{
        ui->Function3Config->show();
    }
}


void MainWindow::on_Function4_clicked()
{
    QWidget* widgetsToHide[] = { ui->Function1Config, ui->Function2Config, ui->Function3Config };
    int size=sizeof(widgetsToHide) / sizeof(QWidget*);
    Hide(widgetsToHide,size);
    if(ui->Function4Config->isVisible())
    {
        ui->Function4Config->hide();
    }
    else{
        ui->Function4Config->show();
    }
}

