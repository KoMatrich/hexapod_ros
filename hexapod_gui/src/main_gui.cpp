#include "main_gui.h"
#include "ui_main_gui.h"


MainGui::MainGui(QWidget *parent)
    : QWidget(parent)
    , ui_(new Ui::MainGui)
{
    ui_->setupUi(this);

    node_handler_.reset(new ros::NodeHandle("~"));

    std::string joint_states_topic;
    std::string joy_topic;
    std::string next_gait_topic;

    int rate;

    node_handler_->param<std::string>("JOINT_STATES_TOPIC", joint_states_topic, "/joint_states");
    node_handler_->param<std::string>("JOY_TOPIC", joy_topic, "/joy_gui");
    node_handler_->param<std::string>("NEXT_GAIT_TOPIC", next_gait_topic, "/next_gait_topic");

    node_handler_->param<int>("RATE", rate, 60);

    // subscribers
    joint_state_sub_ = node_handler_->subscribe<sensor_msgs::JointState>(
        joint_states_topic, 5, &MainGui::updateCallback, this);

    // publishers
    state_joy_pub_ = node_handler_->advertise<sensor_msgs::Joy>(joy_topic, rate);
    next_gait_pub_ = node_handler_->advertise<std_msgs::Int8>(next_gait_topic, rate);

    joyState_.axes.resize(2);
    joyState_.buttons.resize(4);

    ros_timer_ = new QTimer(this);
    connect(ros_timer_, SIGNAL(timeout()), this, SLOT(spinOnce()));
    ros_timer_->start(rate);
}

MainGui::~MainGui()
{
    delete ui_;
}

QString trim_number(float value){
    return QString::number(value, 'f', 2);
}

void MainGui::updateCallback(const sensor_msgs::JointState::ConstPtr& msg){
    auto time = msg->header.stamp;
    double seconds = time.sec % 60 + time.nsec / 1e9;
    auto msg_time = QString::number(seconds, 'f', 2) + "s";
    auto frame_id = QString::fromStdString(msg->header.frame_id);

    ui_->RosTime->setText("Time: "+msg_time);
    ui_->RosFrameId->setText("Frame Id: "+frame_id);

    auto joint_state_load = msg->effort;

    // right
    ui_->RBCoxaLoad->setText(trim_number(joint_state_load[0]));
    ui_->RBFemurLoad->setText(trim_number(joint_state_load[1]));
    ui_->RBTibiaLoad->setText(trim_number(joint_state_load[2]));

    ui_->RMCoxaLoad->setText(trim_number(joint_state_load[3]));
    ui_->RMFemurLoad->setText(trim_number(joint_state_load[4]));
    ui_->RMTibiaLoad->setText(trim_number(joint_state_load[5]));

    ui_->RFCoxaLoad->setText(trim_number(joint_state_load[6]));
    ui_->RFFemurLoad->setText(trim_number(joint_state_load[7]));
    ui_->RFTibiaLoad->setText(trim_number(joint_state_load[8]));

    //left
    ui_->LBCoxaLoad->setText(trim_number(joint_state_load[9]));
    ui_->LBFemurLoad->setText(trim_number(joint_state_load[10]));
    ui_->LBTibiaLoad->setText(trim_number(joint_state_load[11]));

    ui_->LMCoxaLoad->setText(trim_number(joint_state_load[12]));
    ui_->LMFemurLoad->setText(trim_number(joint_state_load[13]));
    ui_->LMTibiaLoad->setText(trim_number(joint_state_load[14]));

    ui_->LFCoxaLoad->setText(trim_number(joint_state_load[15]));
    ui_->LFFemurLoad->setText(trim_number(joint_state_load[16]));
    ui_->LFTibiaLoad->setText(trim_number(joint_state_load[17]));
}

void MainGui::publishStates(){
    joyState_.axes[1] = 0;
    joyState_.axes[1] += 0 ? 1 : ui_->LPushButtonUp->isDown();
    joyState_.axes[1] -= 0 ? 1 : ui_->LPushButtonBack->isDown();

    joyState_.axes[0] = 0;
    joyState_.axes[0] += 0 ? 1 : ui_->LPushButtonLeft->isDown();
    joyState_.axes[0] -= 0 ? 1 : ui_->LPushButtonRight->isDown();

    joyState_.buttons[0] = 0 ? 1 : ui_->RPushButtonA->isDown();
    joyState_.buttons[1] = 0 ? 1 : ui_->RPushButtonB->isDown();
    joyState_.buttons[2] = 0 ? 1 : ui_->RPushButtonX->isDown();
    joyState_.buttons[3] = 0 ? 1 : ui_->RPushButtonY->isDown();

    state_joy_pub_.publish(joyState_);

    next_gait_.data = -1;

    if(ui_->TripodGaitButton->isDown())
        next_gait_.data = 0;
    if(ui_->TetrapodGaitButton->isDown())
        next_gait_.data = 1;
    if(ui_->WaveGaitButton->isDown())
        next_gait_.data = 2;
    if(ui_->RippleGaitButton->isDown())
        next_gait_.data = 3;

    next_gait_pub_.publish(next_gait_);
}

void MainGui::spinOnce(){
    if(ros::ok()){
        publishStates();
        ros::spinOnce();
    }else{
        QApplication::quit();
    }
}
