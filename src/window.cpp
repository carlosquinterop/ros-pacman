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
    ListArrayMap(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/layouts/");
    
    connect(playBtn, &QPushButton::clicked, this, &Window::PlaySlot);
    connect(mapsList, SIGNAL(currentIndexChanged(QString)), maps, SLOT(CreateMap(QString)));
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, int*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*)), glWidget, SLOT(ReceiveMapDataGL(int, int, QImage*, int*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*)));
    connect(refreshTimer, SIGNAL(timeout()), glWidget, SLOT(UpdateSimulationSlot()));
    connect(listenMsg, SIGNAL(UpdatePacmanCommand(int)), glWidget, SLOT(SetPacmanCommand(int)));
    connect(glWidget, SIGNAL(UpdatePacmanPos(QPoint)), this, SLOT(UpdatePacmanPosSlot(QPoint)));
    
    refreshTimer->start(refreshTimeMs);
    container->addWidget(glWidget);
    maps->CreateMap(mapsList->currentText());
    mainLayout->addWidget(mapsList);

    setLayout(mainLayout);
    
    allowPlay = false;
    
    //Initialize ROS node, suscriptions and advertisements
    char **argv;
    int argc = 0;
    ros::init(argc, argv, "pacman_world");
    node = new ros::NodeHandle();
    subscriber = node->subscribe("exampletopic", 100, &ListenMsgThread::callback, listenMsg);
    publisher = node->advertise<pacman::pacmanPos>("pacmanCoord", 100);
     
    listenMsg->start();  
}

void Window::keyPressEvent(QKeyEvent *e)
{
    
}

void Window::ListArrayMap(QString path)
{
    QDir dir(path);
    dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
    QStringList list = dir.entryList();
    for(int i = 0; i < list.size(); i++)
      mapsList->addItem(list.at(i).split(".")[0]);
}

void Window::PlaySlot()
{
    if(playBtn->text() == "Play")
    {
      listenMsg->setWorkingThread(true);
      playBtn->setText("Stop");
      mapsList->setEnabled(false);
    }
    else
    {
      listenMsg->setWorkingThread(false);
      playBtn->setText("Play");
      maps->CreateMap(mapsList->currentText());
      mapsList->setEnabled(true);
    }
    glWidget->TogglePlaying();
}
void Window::UpdatePacmanPosSlot(QPoint pos)
{
    msg.pacmanPos.x = pos.x();
    msg.pacmanPos.y = pos.y();
    publisher.publish(msg);
    //ROS_INFO("%d", msg.pacmanPos.x);
}