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

void MainGui::updateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    auto joint_state_load = msg->effort;

    // right
    ui->RBCoxaLoad->setNum(joint_state_load[0]);
    ui->RBFemurLoad->setNum(joint_state_load[1]);
    ui->RBTibiaLoad->setNum(joint_state_load[2]);

    ui->RMCoxaLoad->setNum(joint_state_load[4]);
    ui->RMFemurLoad->setNum(joint_state_load[5]);
    ui->RMTibiaLoad->setNum(joint_state_load[6]);

    ui->RFCoxaLoad->setNum(joint_state_load[7]);
    ui->RFFemurLoad->setNum(joint_state_load[8]);
    ui->RFTibiaLoad->setNum(joint_state_load[9]);

    //left
    ui->LBCoxaLoad->setNum(joint_state_load[10]);
    ui->LBFemurLoad->setNum(joint_state_load[11]);
    ui->LBTibiaLoad->setNum(joint_state_load[12]);

    ui->LMCoxaLoad->setNum(joint_state_load[13]);
    ui->LMFemurLoad->setNum(joint_state_load[14]);
    ui->LMTibiaLoad->setNum(joint_state_load[15]);

    ui->LFCoxaLoad->setNum(joint_state_load[16]);
    ui->LFFemurLoad->setNum(joint_state_load[17]);
    ui->LFTibiaLoad->setNum(joint_state_load[18]);
}

void MainGui::spinOnce(){
    if(ros::ok()){
        ros::spinOnce();
    }
    else {
        QApplication::quit();
    }
}
