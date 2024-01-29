#include "myscrolltextbar.h"
#include <QLayout>
#include <QScrollBar>
#include <QPlainTextEdit>
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
    plainTextEdit = new QPlainTextEdit(this);
    horizontalLayout->addWidget(plainTextEdit);
    connect(scrollBar,SIGNAL(sliderMoved(int)), this,SLOT(setPlainText(int)));
}

void MyScrollTextBar::setPlainText(int a)
{
    qDebug()<<a;
    float display_number=float(a);
    plainTextEdit->setPlainText(QString::number(display_number,'f',2));
}
