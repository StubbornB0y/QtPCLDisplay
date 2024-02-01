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

    QScrollBar *scrollBar;
    QLineEdit *lineTextEdit;

signals:


private slots:
    void setLineText(int a);
    void setScrollBar(QString a);

private:
    QHBoxLayout *horizontalLayout;

};


#endif // MYSCROLLTEXTBAR_H
