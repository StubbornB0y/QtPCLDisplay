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
    scrollBar->setMaximumSize(QSize(181, 20));
    scrollBar->setOrientation(Qt::Horizontal);
    horizontalLayout->addWidget(scrollBar);
    lineTextEdit = new QLineEdit(this);
    horizontalLayout->addWidget(lineTextEdit);
    connect(scrollBar,SIGNAL(sliderMoved(int)), this,SLOT(setLineText(int)));
    connect(lineTextEdit, SIGNAL(textEdited(QString)), this, SLOT(setScrollBar(QString)));
    connect(lineTextEdit, SIGNAL(textChanged(QString)), this, SLOT(emitSignal(QString)));
}

void MyScrollTextBar::setLineText(int a)
{
    qDebug()<<a;
    if(is_double)
    {
        double display_number=double(a)/scale;
        lineTextEdit->setText(QString::number(display_number,'f',2));
    }
    else
    {
        lineTextEdit->setText(QString::number(a));
    }
}

void MyScrollTextBar::setScrollBar(QString a)
{
    qDebug()<<a;
    bool ok;
    double middleValue;
    int scrollValue;
    
    if(is_double){
        middleValue = a.toDouble(&ok);
        if(!ok){
            qDebug()<<"Invalid input!";
        }
        else{
            scrollValue = int(middleValue*scale);
            scrollBar->setValue(scrollValue);
        }
    }
    else{
        scrollValue = a.toInt(&ok);
        if(!ok){
            qDebug()<<"Invalid input!";
        }
        else{
            scrollBar->setValue(scrollValue);
        }
    }
    
    /*如果有需要溢出监测 取消注释此段并替换
    if(scrollValue < scrollBar->minimum() || scrollValue > scrollBar->maximum()){
        qDebug()<<"Out of range!!";
    }
    else{
        scrollBar->setValue(scrollValue);
    }*/
    

}

void MyScrollTextBar::setDouble(bool enabled){
    is_double = enabled;
    if(is_double){

    }
}

void MyScrollTextBar::emitSignal(QString text){
    bool ok;
    if(is_double)
    {
        double transValue_double = text.toDouble(&ok);
        if(ok)
        {
            emit changedSignal_double(transValue_double);
            qDebug()<<"double have been emited";
        }
        else
        {
            qDebug()<<"Transform error, invalid input!";
        }
    }
    else{
        int transValue_int = text.toInt(&ok);
        if(ok)
        {
            emit changedSignal_int(transValue_int);
            qDebug()<<"int have been emited";
        }
        else
        {
            qDebug()<<"Transform error, invalid input!";
        }
    }
}

void MyScrollTextBar::setRange(double min_value, double max_value)
{
    scrollBar->setRange(int(min_value * scale), int(max_value * scale));
}

