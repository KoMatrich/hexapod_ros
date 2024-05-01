#include <QApplication>
#include <QIcon>
#include "main_gui.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_gui");
    QApplication a(argc, argv);

    MainGui main_window;

    main_window.setWindowTitle(QString::fromStdString("Hexapod Control Panel"));

    QIcon icon(":/icon.ico");
    main_window.setWindowIcon(icon);

    main_window.show();
    return a.exec();
}
