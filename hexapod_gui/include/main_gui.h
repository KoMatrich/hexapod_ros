#ifndef MAIN_GUI_H
#define MAIN_GUI_H

#include <QWidget>
#include <ros/ros.h>
#include <qtimer.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

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
    Ui::MainGui *ui_;

    QTimer *ros_timer_;

    ros::NodeHandlePtr node_handler_;

    ros::Subscriber joint_state_sub_;

    ros::Publisher state_joy_pub_;
    ros::Publisher next_gait_pub_;

    sensor_msgs::Joy joyState_;
    std_msgs::Int8 next_gait_;

    void updateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void publishStates();
};

#endif // MAIN_GUI_H
