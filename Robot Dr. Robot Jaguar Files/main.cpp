#include <QtGui>
#include <QApplication>
#include "../include/drrobot_jaguar_v6/mainwindow.hpp"
//#include "../include/drrobot_jaguar_v6/qnode.hpp"

using namespace drrobot_jaguar_v6;

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);

    MainWindow Mw(argc,argv);
    Mw.connectToRobot();

    QObject::connect(&Mw, SIGNAL(CloseProg()), &app, SLOT(quit()), Qt::QueuedConnection);

    return app.exec();
}

