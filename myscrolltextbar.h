#ifndef MYSCROLLTEXTBAR_H
#define MYSCROLLTEXTBAR_H
#include <QLayout>
#include <QScrollBar>
#include <QLineEdit>
#include <QGroupBox>

class MyScrollTextBar : public QGroupBox
{
    Q_OBJECT
public:
    explicit MyScrollTextBar(QWidget *parent = nullptr);

    void setDouble(bool enabled);

    void setScale(double Scale){this->scale=Scale;}
    const double getScale(){return scale;}

    void setRange(double min_value, double max_value);
    QScrollBar *scrollBar;
    QLineEdit *lineTextEdit;

signals:
    void changedSignal_double(double a);
    void changedSignal_int(int a);

private slots:
    void emitSignal(QString text);
    void setLineText(int a);
    void setScrollBar(QString a);

private:

    bool is_double = false;     //设置是int显示还是double显示
    double scale = 100;   //如果启用double表示，为scrollbar的int和Linetext的double文本做比例切换 默认100 
    QHBoxLayout *horizontalLayout;

};


#endif // MYSCROLLTEXTBAR_H
