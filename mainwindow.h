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
