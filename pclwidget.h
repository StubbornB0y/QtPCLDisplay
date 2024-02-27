#ifndef PCLWIDGET_H
#define PCLWIDGET_H

#include <QWidget>
#include <QVTKOpenGLStereoWidget.h>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
class PCLWidget : public QVTKOpenGLStereoWidget
{
    Q_OBJECT
public:
    explicit PCLWidget(QWidget *parent = nullptr);

private:
    vtkSmartPointer<vtkRenderer> m_render;

signals:
};

#endif // PCLWIDGET_H
