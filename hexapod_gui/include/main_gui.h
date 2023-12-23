#ifndef MAIN_GUI_H
#define MAIN_GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>

namespace Ui {
class MainGui;
}

class MainGui : public QWidget
{
    Q_OBJECT

public:
    explicit MainGui(QWidget *parent = nullptr);
    ~MainGui();

public slots:
    void spinOnce();

private:
    Ui::MainGui *ui;

    QTimer *ros_timer;
};

#endif // MAIN_GUI_H
