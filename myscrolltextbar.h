#ifndef MYSCROLLTEXTBAR_H
#define MYSCROLLTEXTBAR_H
#include <QLayout>
#include <QScrollBar>
#include <QPlainTextEdit>
#include <QGroupBox>

class MyScrollTextBar : public QGroupBox
{
    Q_OBJECT
public:
    explicit MyScrollTextBar(QWidget *parent = nullptr);

    QScrollBar *scrollBar;
    QPlainTextEdit *plainTextEdit;

signals:


private slots:
    void setPlainText(int a);

private:
    QHBoxLayout *horizontalLayout;

};


#endif // MYSCROLLTEXTBAR_H
