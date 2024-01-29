#include "pclwidget.h"

#include <vtkActor.h>
#include <vtkNamedColors.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>


PCLWidget::PCLWidget(QWidget *parent)
    : QVTKOpenGLStereoWidget{parent}
{
    vtkNew<vtkNamedColors> colors;
    // 创建渲染器实例并设置他的背景色
    m_render = vtkSmartPointer<vtkRenderer>::New();
    m_render->SetBackground(colors->GetColor3d("Black").GetData());
    this->renderWindow()->AddRenderer(m_render);

    // 创建球体
    vtkNew<vtkSphereSource> sphereSource;
    sphereSource->SetCenter(0.0, 0.0, 0.0);
    sphereSource->SetRadius(5.0);
    sphereSource->SetPhiResolution(100);
    sphereSource->SetThetaResolution(100);
    // 创建mapper
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(sphereSource->GetOutputPort());

    // 创建actor
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(colors->GetColor3d("Cornsilk").GetData());
    // 将actor加载到渲染器中
    m_render->AddActor(actor);
}
