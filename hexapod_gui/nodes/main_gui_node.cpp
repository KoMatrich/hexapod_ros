#include <QApplication>
#include <QIcon>
#include "main_gui.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_gui", ros::init_options::AnonymousName);
    QApplication a(argc, argv);

    MainGui w;

    w.setWindowTitle(QString::fromStdString(
        "Qt UI"));

    QIcon icon(":/icons/my_gui_icon.png");
    w.setWindowIcon(icon);

    w.show();
    return a.exec();
}
