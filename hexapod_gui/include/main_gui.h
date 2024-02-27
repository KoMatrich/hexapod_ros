#ifndef MAIN_GUI_H
#define MAIN_GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <sensor_msgs/JointState.h>

namespace Ui {
class MainGui;
}

class MainGui : public QWidget
{
    Q_OBJECT

public:
    explicit MainGui(QWidget *parent = nullptr);
    ~MainGui();

    void updateCallback(const sensor_msgs::JointState::ConstPtr& msg);

public slots:
    void spinOnce();

private:
    Ui::MainGui *ui;

    QTimer *ros_timer;

    ros::NodeHandlePtr nh_;
    ros::Subscriber sub_;
};

#endif // MAIN_GUI_H
