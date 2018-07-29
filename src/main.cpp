#include <QApplication>
#include "pacman/window.h"
#include "ros/ros.h"
#include "pacman/pacmanPos.h"
#include "pacman/pos.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QStringList args = app.arguments();	//GED Jul-27: Los argumentos en linea de comandos son guardados en la QStrilList args
    Window mainWindow(args);		//GED Jul-27: Se invoca mainWndow con los argumentos capturados.
    //mainWindow.resize(mainWindow.sizeHint());
    mainWindow.show();
    return app.exec();
}
