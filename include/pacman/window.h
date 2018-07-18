#ifndef WINDOW_H
#define WINDOW_H

#include <QWidget>
#include <QObject>
#include "maps.h"
#include <iostream>
#include <ros/package.h>
#include "pacman/glwidget.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>

using namespace std;

class QSlider;
class QPushButton;

class GLWidget;
class MainWindow;

class Window : public QWidget
{
    Q_OBJECT

public:
    Window();

private:
	void listArrayMap(QString path);

protected:
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void playSlot();
    
private:
    GLWidget *glWidget;
    QPushButton *playBtn;
    MainWindow *mainWindow;
    Maps *maps;
	QComboBox *mapsList;
	QHBoxLayout *container;
    bool allowPlay;
    
signals:
    void arrowKey(int key);
};

#endif
