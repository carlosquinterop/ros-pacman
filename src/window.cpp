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
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*)), glWidget, SLOT(ReceiveMapDataGL(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*)));
    connect(refreshTimer, SIGNAL(timeout()), glWidget, SLOT(UpdateSimulationSlot()));
    connect(listenMsg, SIGNAL(UpdatePacmanCommand(int)), glWidget, SLOT(SetPacmanCommand(int)));
    connect(glWidget, SIGNAL(UpdatePacmanPos(QPoint*, int)), this, SLOT(UpdatePacmanPosSlot(QPoint*, int)));
    connect(glWidget, SIGNAL(UpdateGhostsPos(QVector<QPoint>*, int)), this, SLOT(UpdateGhostsPosSlot(QVector<QPoint>*, int)));
    connect(glWidget, SIGNAL(UpdateCookiesPos(QVector<QPoint>*, int)), this, SLOT(UpdateCookiesPosSlot(QVector<QPoint>*, int)));
    connect(glWidget, SIGNAL(UpdateBonusPos(QVector<QPoint>*, int)), this, SLOT(UpdateBonusPosSlot(QVector<QPoint>*, int)));
    
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
void Window::UpdatePacmanPosSlot(QPoint* pos, int nPacman)
{
  msgPacman.pacmanPos.resize(nPacman);
  for(int i = 0; i < nPacman; i++)
  {
    msgPacman.pacmanPos[i].x = pos[i].x(); 
    msgPacman.pacmanPos[i].y = pos[i].y();
  }
  msgPacman.nPacman = nPacman;
  pacmanPublisher.publish(msgPacman);
}
void Window::UpdateGhostsPosSlot(QVector<QPoint>* pos, int nGhosts)
{
  msgGhosts.ghostsPos.resize(nGhosts);
  for(int i = 0; i < nGhosts; i++)
  {
    msgGhosts.ghostsPos[i].x = pos->at(i).x(); 
    msgGhosts.ghostsPos[i].y = pos->at(i).y();
  }
  msgGhosts.nGhosts = nGhosts;
  ghostPublisher.publish(msgGhosts);
}
void Window::UpdateCookiesPosSlot(QVector<QPoint>* pos, int nCookies)
{
  msgCookies.cookiesPos.resize(nCookies);
  for(int i = 0; i < nCookies; i++)
  {
    msgCookies.cookiesPos[i].x = pos->at(i).x();
    msgCookies.cookiesPos[i].y = pos->at(i).y();
  }
  msgCookies.nCookies = nCookies;
  cookiesPublisher.publish(msgCookies);
}
void Window::UpdateBonusPosSlot(QVector<QPoint>* pos, int nBonus)
{
  msgBonus.bonusPos.resize(nBonus);
  for(int i = 0; i < nBonus; i++)
  {
    msgBonus.bonusPos[i].x = pos->at(i).x();
    msgBonus.bonusPos[i].y = pos->at(i).y();
  }
  msgBonus.nBonus = nBonus;
  bonusPublisher.publish(msgBonus);
  //printf("nBonus : %d \n",nBonus);
}