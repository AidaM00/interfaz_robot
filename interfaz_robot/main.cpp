#include "interfaz_robot.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    interfaz_robot window;
    window.show();
    return app.exec();
}
