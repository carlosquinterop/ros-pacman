#include <QApplication>
#include "pacman/window.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    /*
    MainWindow mainWindow;
 	mainWindow.resize(mainWindow.sizeHint());
    mainWindow.show();
    return app.exec();
    
    */
    Window mainWindow;
 	mainWindow.resize(mainWindow.sizeHint());
    mainWindow.show();
    return app.exec();
}
