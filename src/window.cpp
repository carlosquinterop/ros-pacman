#include "pacman/window.h"


Window::Window()
{
    maps = new Maps;
    glWidget = new GLWidget;
    refreshTimer = new QTimer();
    mainLayout = new QVBoxLayout;
    container = new QHBoxLayout;
    listenMsg = new ListenMsgThread();
       
    QWidget *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);
    playBtn = new QPushButton(tr("Play"));
    mainLayout->addWidget(playBtn);
    
    mapsList = new QComboBox;    
    listArrayMap(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/layouts/");
    
    connect(playBtn, &QPushButton::clicked, this, &Window::playSlot);
    connect(mapsList, SIGNAL(currentIndexChanged(QString)), maps, SLOT(createMap(QString)));
    connect(maps, SIGNAL(sendMapData(int,int,QImage*,bool*,int,int)), glWidget, SLOT(receiveMapDataGL(int,int,QImage*,bool*,int,int)));
    connect(refreshTimer, SIGNAL(timeout()), glWidget, SLOT(updateSimulationSlot()));
    connect(listenMsg, SIGNAL(UpdatePacmanCommand(int)), glWidget, SLOT(setPacmanCommand(int)));
    
    refreshTimer->start(refreshTimeMs);
    
    container->addWidget(glWidget);
    maps->createMap(mapsList->currentText());
    mainLayout->addWidget(mapsList);

    setLayout(mainLayout);
    
    allowPlay = false;
}

void Window::keyPressEvent(QKeyEvent *e)
{
    
}

void Window::listArrayMap(QString path)
{
    QDir dir(path);
    dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
    QStringList list = dir.entryList();
    for(int i = 0; i < list.size(); i++)
	mapsList->addItem(list.at(i));
}

void Window::playSlot()
{
    if(playBtn->text() == "Play")
    {
      playBtn->setText("Stop");
      mapsList->setEnabled(false);
      listenMsg->start();
    }
    else
    {
      playBtn->setText("Play");
      listenMsg->setWorkingThread();
      maps->createMap(mapsList->currentText());
      mapsList->setEnabled(true);
    }
}