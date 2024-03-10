#include "main_gui.h"
#include "ui_main_gui.h"


MainGui::MainGui(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainGui)
{
    ui->setupUi(this);

    nh_.reset(new ros::NodeHandle("~"));

    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    ros_timer->start(100);

    std::string joint_states_topic;
    nh_->param<std::string>("JOINT_STATES_TOPIC", joint_states_topic, "/joint_states");
    sub_ = nh_->subscribe<sensor_msgs::JointState>(joint_states_topic, 1, &MainGui::updateCallback, this);
}

MainGui::~MainGui()
{
    delete ui;
}

QString trim_number(float value){
    return QString::number(value, 'f', 2);
}

void MainGui::updateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    auto msg_time = QString::number(msg->header.stamp.nsec*1.0/(10^9),'f',2) + "s";
    auto frame_id = msg->header.frame_id;

    ui->RosTime->setText("Time: "+msg_time);
    ui->RosFrameId->setText("Frame Id: "+frame_id);

    auto joint_state_load = msg->effort;

    // right
    ui->RBCoxaLoad->setText(trim_number(joint_state_load[0]));
    ui->RBFemurLoad->setText(trim_number(joint_state_load[1]));
    ui->RBTibiaLoad->setText(trim_number(joint_state_load[2]));

    ui->RMCoxaLoad->setText(trim_number(joint_state_load[3]));
    ui->RMFemurLoad->setText(trim_number(joint_state_load[4]));
    ui->RMTibiaLoad->setText(trim_number(joint_state_load[5]));

    ui->RFCoxaLoad->setText(trim_number(joint_state_load[6]));
    ui->RFFemurLoad->setText(trim_number(joint_state_load[7]));
    ui->RFTibiaLoad->setText(trim_number(joint_state_load[8]));

    //left
    ui->LBCoxaLoad->setText(trim_number(joint_state_load[9]));
    ui->LBFemurLoad->setText(trim_number(joint_state_load[10]));
    ui->LBTibiaLoad->setText(trim_number(joint_state_load[11]));

    ui->LMCoxaLoad->setText(trim_number(joint_state_load[12]));
    ui->LMFemurLoad->setText(trim_number(joint_state_load[13]));
    ui->LMTibiaLoad->setText(trim_number(joint_state_load[14]));

    ui->LFCoxaLoad->setText(trim_number(joint_state_load[15]));
    ui->LFFemurLoad->setText(trim_number(joint_state_load[16]));
    ui->LFTibiaLoad->setText(trim_number(joint_state_load[17]));
}

void MainGui::spinOnce(){
    if(ros::ok()){
        ros::spinOnce();
    }
    else {
        QApplication::quit();
    }
}
