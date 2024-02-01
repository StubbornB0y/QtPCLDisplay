#include "myscrolltextbar.h"
#include <QLayout>
#include <QScrollBar>
#include <QLineEdit>
#include <QString>
#include <QDebug>
MyScrollTextBar::MyScrollTextBar(QWidget *parent) : QGroupBox(parent)
{

    this->setBaseSize(281,81);
    horizontalLayout = new QHBoxLayout(this);
    scrollBar = new QScrollBar(this);
    scrollBar->setMinimumSize(QSize(181, 20));
    scrollBar->setBaseSize(QSize(0, 0));
    scrollBar->setOrientation(Qt::Horizontal);
    horizontalLayout->addWidget(scrollBar);
    lineTextEdit = new QLineEdit(this);
    horizontalLayout->addWidget(lineTextEdit);
    connect(scrollBar,SIGNAL(sliderMoved(int)), this,SLOT(setLineText(int)));
    connect(lineTextEdit,SIGNAL(textEdited(QString)),this,SLOT(setScrollBar(QString)));
}

void MyScrollTextBar::setLineText(int a)
{
    qDebug()<<a;
    float display_number=float(a);
    lineTextEdit->setText(QString::number(display_number,'f',2));
}

void MyScrollTextBar::setScrollBar(QString a)
{
    qDebug()<<a;
    bool ok;
    float middleValue = a.toFloat(&ok);

    if(!ok){
        qDebug()<<"Invalid input!";
    }
    else{
        int scrollValue=int(middleValue);
        if(scrollValue<=0 || scrollValue>99){
            qDebug()<<"Out of range!!";
        }
        else{
            scrollBar->setValue(scrollValue);
        }
    }

}
