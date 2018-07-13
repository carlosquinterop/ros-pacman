#include "pacman/glwidget.h"
#include "pacman/window.h"
#include "pacman/mainwindow.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QKeyEvent>
#include <QPushButton>
#include <QDesktopWidget>
#include <QApplication>
#include <QMessageBox>

Window::Window(MainWindow *mw)
    : mainWindow(mw)
{
    glWidget = new GLWidget;

    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *container = new QHBoxLayout;
    container->addWidget(glWidget);
   
    QWidget *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);
    playBtn = new QPushButton(tr("Play"));
    connect(playBtn, &QPushButton::clicked, this, &Window::playSlot);
    mainLayout->addWidget(playBtn);

    setLayout(mainLayout);
    
    connect(this, SIGNAL(arrowKey(int)), glWidget, SLOT(receiveKeySlot(int)));
}

void Window::keyPressEvent(QKeyEvent *e)
{
    if (e->key() == Qt::Key_Escape)
        close();
    else 
    {
	emit arrowKey(e->key());
        //QWidget::keyPressEvent(e);
	//cout << "algo" << endl;   
    }
}

void Window::playSlot()
{
   
}
