#include "pacman/window.h"


Window::Window(QStringList args)	//GED Jul-27: Se recibe QStringList args con  argumentos
{
    char **argv;
    int argc = 0;
    ros::init(argc, argv, "pacman_world");
    QString mapName;
    QWidget *w = new QWidget();
    QWidget *wScores = new QWidget();
    wScores->setFixedSize(460, 60);
    maps = new Maps();
    
    scoreName = new QLabel("Score: ");
    scoreLabel = new QLabel();
    livesName = new QLabel("Lives: ");
    livesLabel = new QLabel();
    
    glWidget = new GLWidget();
    refreshTimer = new QTimer();
    mainLayout = new QVBoxLayout();
    container = new QHBoxLayout();
    containerScores = new QHBoxLayout();
    listenMsg = new ListenMsgThread();
    mapsList = new QComboBox();
    counterTimer = new QTimer();	//GED Jul-27
    playBtn = new QPushButton(tr("Play"));
    counterBtn = new QPushButton(tr("Ready player one?")); //GED Jul-27: PushButton label for reverse counter
    node = new ros::NodeHandle();    
    
    w->setLayout(container);
    wScores->setLayout(containerScores);
    mainLayout->addWidget(wScores);
    mainLayout->addWidget(w);
    counterBtn->setEnabled(false);
    container->addWidget(glWidget);
    containerScores->addWidget(scoreName);  
    containerScores->addWidget(scoreLabel);
    containerScores->addWidget(livesName);
    containerScores->addWidget(livesLabel);
    ListArrayMap(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/layouts/");
    allowPlay = false;
   
    mode = getArguments(args);//GED Jul-28
    if(mode == 1)  		//GED Jul-27: if Game mode
    {
      mainLayout->addWidget(playBtn);
      mainLayout->addWidget(mapsList);
      mapName = mapsList->currentText();
      connect(this, SIGNAL(ArrowKey(int)), glWidget, SLOT(receiveArrowKey(int)));
    }
    else if(mode == 2)
    {
      mainLayout->addWidget(counterBtn);
      counterTimer->start(1000);		//GED Jul-27
      mapName = args.at(2);
    }
    else
    {
      cout << "Error: Pacman mode argument missing" << endl;
      cout << "Usage: rosrun pacman pacman_world [pacmanMode] {opt:mapName}" << endl;
      cout << "     pacmanMode : test" << endl;
      cout << "                : game" << endl;
      cout << "     mapName    : file name of the map" << endl;
      exit(0);
    }
    
    connect(playBtn, &QPushButton::clicked, this, &Window::PlaySlot);
    connect(mapsList, SIGNAL(currentIndexChanged(QString)), maps, SLOT(CreateMap(QString)));
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, int, int)), glWidget, SLOT(ReceiveMapDataGL(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, int, int)));
    connect(maps, SIGNAL(SendMapData(int, int, QImage*, bool*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, QVector<int>*, int, int)), this, SLOT(UpdateSizeSlot()));
    connect(refreshTimer, SIGNAL(timeout()), glWidget, SLOT(UpdateSimulationSlot()));
    connect(listenMsg, SIGNAL(UpdatePacmanCommand(int)), glWidget, SLOT(SetPacmanCommand(int)));
    connect(glWidget, SIGNAL(UpdatePacmanPos(QVector<QPoint>*)), this, SLOT(UpdatePacmanPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateGhostsPos(QVector<QPoint>*, bool*)), this, SLOT(UpdateGhostsPosSlot(QVector<QPoint>*, bool*)));
    connect(glWidget, SIGNAL(UpdateCookiesPos(QVector<QPoint>*)), this, SLOT(UpdateCookiesPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateBonusPos(QVector<QPoint>*)), this, SLOT(UpdateBonusPosSlot(QVector<QPoint>*)));
    connect(glWidget, SIGNAL(UpdateObstaclesPos(QVector<QPoint>*, int, int, int, int)), this, SLOT(UpdateObstaclesPosSlot(QVector<QPoint>*, int, int, int, int)));
    connect(glWidget, SIGNAL(DeadPacmanSignal()), this, SLOT(DeadPacmanSlot()));
    connect(glWidget, SIGNAL(EndOfDeadPacmanSignal()), this, SLOT(EndOfDeadPacmanSlot()));
    connect(glWidget, SIGNAL(UpdateScores(int, int)), this, SLOT(UpdateScoresSlot(int, int)));
    connect(counterTimer, SIGNAL(timeout()), this, SLOT(timerFunction()));	//GED Jul-27
    connect(glWidget, SIGNAL(updateGameState()), this, SLOT(UpdateGameStateSlot()));   
    setLayout(mainLayout);
    this->setMaximumSize(QSize(maxWidth, maxHeight));
    maps->CreateMap(mapName);
    subscriber = node->subscribe("pacmanActions", 100, &ListenMsgThread::callback, listenMsg);
    pacmanPublisher = node->advertise<pacman::pacmanPos>("pacmanCoord", 100);
    ghostPublisher = node->advertise<pacman::ghostsPos>("ghostsCoord",100);
    cookiesPublisher = node->advertise<pacman::cookiesPos>("cookiesCoord",100);
    bonusPublisher = node->advertise<pacman::bonusPos>("bonusCoord",100); 
    gameStatePublisher = node->advertise<pacman::game>("gameState",100);
    mapResponseServer = node->advertiseService("pacman_world", &Window::obsService, this);
    
    listenMsg->start();
    refreshTimer->start(refreshTimeMs);
}

QSize Window::sizeHint() const
{
    return QSize(maps->getWidth(), maps->getHeight());
}

QSize Window::minimumSizeHint() const
{
    return QSize(100, 100);
}

int Window::getArguments(QStringList args)				//GED Jul-28
{
    int pacmanMode = 0;
    int pacmanGameMode = 1, pacmanTestMode = -1;
    
    for (QStringList::iterator it = args.begin(); it != args.end(); ++it)//GED Jul-28
    {
      QString current = *it;
      QString pacmanModeG="game";
      QString pacmanModeT="test";
      pacmanGameMode = QString::compare(pacmanModeG, current, Qt::CaseInsensitive)*pacmanGameMode;
      pacmanTestMode = QString::compare(pacmanModeT, current, Qt::CaseInsensitive)*pacmanTestMode;
    }
    
    if(pacmanGameMode == 0)
      pacmanMode = 1; //GED Jul-28: Game mode
    else if(pacmanTestMode == 0)
      pacmanMode = 2; //GED Jul-28: Test mode
    else
      pacmanMode = 0;
    
    return pacmanMode;
}

void Window::timerFunction() 				//GED Jul-27
{
  counterBtn->setStyleSheet("background-color: black;"
                            "font: bold 28px;"
			    "color: yellow;"
			   );      
  if(counterBtn->text() == "Ready player one?")
    {
      counterBtn->setText("Playing in... 3");
    }
  else if(counterBtn->text() == "Playing in... 3")
    {
      counterBtn->setText("Playing in... 2");
    }
    
  else if(counterBtn->text() == "Playing in... 2")
    {
      counterBtn->setText("Playing in... 1");
    }
  else if(counterBtn->text() == "Playing in... 1")
    {
      counterTimer->stop();
      counterBtn->setText("Play!");
      listenMsg->setWorkingThread(true);
      glWidget->TogglePlaying();
      gameState = true;
    }
}

void Window::keyPressEvent(QKeyEvent *e)
{
    //if (mode == 1 && allowPlay)
    if (allowPlay)
      emit ArrowKey(e->key());
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
      allowPlay = true;
    }
    else
    {
      listenMsg->setWorkingThread(false);
      playBtn->setText("Play");
      maps->CreateMap(mapsList->currentText());
      mapsList->setEnabled(true);
      allowPlay = false;
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
void Window::UpdateGhostsPosSlot(QVector<QPoint>* pos, bool* ghostsMode)
{
  msgGhosts.ghostsPos.resize(pos->size());
  msgGhosts.mode.resize(pos->size());
  for(int i = 0; i < pos->size(); i++)
  {
    msgGhosts.ghostsPos[i].x = pos->at(i).x(); 
    msgGhosts.ghostsPos[i].y = pos->at(i).y();
    msgGhosts.mode[i] = (int)ghostsMode[i];
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

void Window::UpdateObstaclesPosSlot(QVector< QPoint >* pos, int xMin, int xMax, int yMin, int yMax)
{
  posObstacles = pos;
  minX = xMin;
  maxX = xMax;
  minY = yMin;
  maxY = yMax;
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

void Window::UpdateGameStateSlot()
{
  msgState.state = (int)gameState; 
  gameStatePublisher.publish(msgState);
}

void Window::UpdateScoresSlot(int score, int lives)
{
  scoreLabel->setText(QString::number(score));
  livesLabel->setText(QString::number(lives));
}


bool Window::obsService(pacman::mapService::Request& req, pacman::mapService::Response& res)
{
  res.minX = minX;
  res.maxX = maxX;
  res.minY = minY;
  res.maxY = maxY;
  res.nObs = posObstacles->size();
  res.obs.resize(posObstacles->size());
  for(int i = 0; i < posObstacles->size(); i++)
  {
    res.obs[i].x = posObstacles->at(i).x();
    res.obs[i].y = posObstacles->at(i).y();
  }
  return true;
}
