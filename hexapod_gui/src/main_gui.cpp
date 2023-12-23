#include "main_gui.h"
#include "ui_main_gui.h"

MainGui::MainGui(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::MainGui)
{
    ui->setupUi(this);

    ros_timer = new QTimer(this);
    connect(ros_timer, SIGNAL(timeout()), this, SLOT(spinOnce()));
    ros_timer->start(100);
}

MainGui::~MainGui()
{
    delete ui;
}

void MainGui::spinOnce(){
    if(ros::ok()){
        ros::spinOnce();
    }
    else {
        QApplication::quit();
    }
}
