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
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*)), glWidget, SLOT(ReceiveMapDataGL(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*)));
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*)), this, SLOT(UpdateSizeSlot()));
    connect(refreshTimer, SIGNAL(timeout()), glWidget, SLOT(UpdateSimulationSlot()));
    connect(listenMsg, SIGNAL(UpdatePacmanCommand(int)), glWidget, SLOT(SetPacmanCommand(int)));
    connect(glWidget, SIGNAL(UpdatePacmanPos(QVector<QPoint>*)), this, SLOT(UpdatePacmanPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateGhostsPos(QVector<QPoint>*)), this, SLOT(UpdateGhostsPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateCookiesPos(QVector<QPoint>*)), this, SLOT(UpdateCookiesPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateBonusPos(QVector<QPoint>*)), this, SLOT(UpdateBonusPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateObstaclesPos(QVector<QPoint>*)), this, SLOT(UpdateObstaclesPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(DeadPacmanSignal()), this, SLOT(DeadPacmanSlot()));
    connect(glWidget, SIGNAL(EndOfDeadPacmanSignal()), this, SLOT(EndOfDeadPacmanSlot()));
    
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
    subscriber = node->subscribe("pacmanActions", 100, &ListenMsgThread::callback, listenMsg);
    pacmanPublisher = node->advertise<pacman::pacmanPos>("pacmanCoord", 100);
    ghostPublisher = node->advertise<pacman::ghostsPos>("ghostsCoord",100);
    cookiesPublisher = node->advertise<pacman::cookiesPos>("cookiesCoord",100);
    bonusPublisher = node->advertise<pacman::bonusPos>("bonusCoord",100); 
    listenMsg->start(); 
    this->setMaximumSize(QSize(maxWidth, maxHeight));
}

QSize Window::sizeHint() const
{
    return QSize(maps->getWidth(), maps->getHeight());
}

QSize Window::minimumSizeHint() const
{
    return QSize(100, 100);
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
void Window::UpdatePacmanPosSlot(QVector<QPoint>* pos)
{
  msgPacman.pacmanPos.resize(pos->size());
  for(int i = 0; i < pos->size(); i++)
  {
    msgPacman.pacmanPos[i].x = pos->at(i).x(); 
    msgPacman.pacmanPos[i].y = pos->at(i).y();
  }
  msgPacman.nPacman = pos->size();
  pacmanPublisher.publish(msgPacman);
}
void Window::UpdateGhostsPosSlot(QVector<QPoint>* pos)
{
  msgGhosts.ghostsPos.resize(pos->size());
  for(int i = 0; i < pos->size(); i++)
  {
    msgGhosts.ghostsPos[i].x = pos->at(i).x(); 
    msgGhosts.ghostsPos[i].y = pos->at(i).y();
  }
  msgGhosts.nGhosts = pos->size();
  ghostPublisher.publish(msgGhosts);
}
void Window::UpdateCookiesPosSlot(QVector<QPoint>* pos)
{
  msgCookies.cookiesPos.resize(pos->size());
  for(int i = 0; i < pos->size(); i++)
  {
    msgCookies.cookiesPos[i].x = pos->at(i).x();
    msgCookies.cookiesPos[i].y = pos->at(i).y();
  }
  msgCookies.nCookies = pos->size();
  cookiesPublisher.publish(msgCookies);
}
void Window::UpdateBonusPosSlot(QVector<QPoint>* pos)
{
  msgBonus.bonusPos.resize(pos->size());
  for(int i = 0; i < pos->size(); i++)
  {
    msgBonus.bonusPos[i].x = pos->at(i).x();
    msgBonus.bonusPos[i].y = pos->at(i).y();
  }
  msgBonus.nBonus = pos->size();
  bonusPublisher.publish(msgBonus);
}

void Window::UpdateObstaclesPosSlot(QVector< QPoint >* pos)
{
  /*msgObstacles.obstaclesPos.resize(pos->size());
  for(int i = 0; i < pos->size(); i++)
  {
    msgObstacles.obstaclesPos[i].x = pos->at(i).x();
    msgObstacles.obstaclesPos[i].y = pos->at(i).y();
  }
  msgObstacles.nObstacles = pos->size();
  obstaclesPublisher.publish(msgObstacles);*/
}

void Window::UpdateSizeSlot()
{
    this->resize(sizeHint());
}

void Window::DeadPacmanSlot()
{
    refreshTimer->stop();
}

void Window::EndOfDeadPacmanSlot()
{
    refreshTimer->start(refreshTimeMs);
}
