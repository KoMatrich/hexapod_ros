#include <QApplication>
#include <QIcon>
#include "main_gui.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_gui", ros::init_options::AnonymousName);
    QApplication a(argc, argv);

    MainGui main_window;

    main_window.setWindowTitle(QString::fromStdString(
        "Qt UI"));

    QIcon icon(":/icons/my_gui_icon.png");
    main_window.setWindowIcon(icon);

    main_window.show();
    return a.exec();
}
