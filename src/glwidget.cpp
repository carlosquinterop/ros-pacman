#include "pacman/glwidget.h"

GLWidget::GLWidget(QWidget *parent)
{
    firstTime = true;
    nPacman = 1;
    nGhosts = 0;
}

GLWidget::~GLWidget()
{

}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(mapWidth, mapHeight);
}

void GLWidget::loadTexture (QImage* img)
{
    GLuint tex;
    glEnable(GL_TEXTURE_2D); // Enable texturing
    QImage t = (img->convertToFormat(QImage::Format_RGBA8888)).mirrored();
    glGenTextures(1, &tex); // Obtain an id for the texture
    glBindTexture(GL_TEXTURE_2D, tex); // Set as the current texture
    texIds.append(tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, t.width(), t.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, t.bits());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDisable(GL_TEXTURE_2D);
}

void GLWidget::loadNewTexture (QImage* img)
{
    GLuint tex;
    glEnable(GL_TEXTURE_2D); // Enable texturing
    QImage t = (img->convertToFormat(QImage::Format_RGBA8888)).mirrored();
    glGenTextures(1, &tex); // Obtain an id for the texture
    glBindTexture(GL_TEXTURE_2D, tex); // Set as the current texture
    texIds[17] = tex;	
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, t.width(), t.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, t.bits());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDisable(GL_TEXTURE_2D);
}

void GLWidget::drawMap()
{
    glColor3f(0.7, 0.7, 0.7);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texIds[17]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f  (-0.5*mapWidth, -0.5*mapHeight);
    glTexCoord2f(0.0f,  1.0f);
    glVertex2f  (-0.5*mapWidth,  0.5*mapHeight);
    glTexCoord2f(1.0f,  1.0f);
    glVertex2f  (0.5*mapWidth,  0.5*mapHeight);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f  (0.5*mapWidth, -0.5*mapHeight);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void GLWidget::drawPacman()
{
    for(int i=0;i < nPacman;i++)
    {
	glShadeModel(GL_SMOOTH);
	glPushMatrix();
	glTranslated(pacmanArray[i]->currentPosition.x(),pacmanArray[i]->currentPosition.y(), 0);
	glRotated(pacmanArray[i]->orientation, 0, 0, 1);
	glColor3f(1.0, 1.0, 1.0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texIds[0]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f  (-0.5*pacmanArray[i]->width, -0.5*pacmanArray[i]->height);
	glTexCoord2f(0.0f,  1.0f);
	glVertex2f  (-0.5*pacmanArray[i]->width,  0.5*pacmanArray[i]->height);
	glTexCoord2f(1.0f,  1.0f);
	glVertex2f  (0.5*pacmanArray[i]->width,  0.5*pacmanArray[i]->height);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f  (0.5*pacmanArray[i]->width, -0.5*pacmanArray[i]->height);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	glColor3f(1.0, 1.0, 1.0);
    }
}

void GLWidget::drawGhosts()
{
    for(int i=0;i < nGhosts;i++)
    {
	glShadeModel(GL_SMOOTH);
	glPushMatrix();
	glTranslated(ghostsArray[i]->currentPosition.x(), ghostsArray[i]->currentPosition.y(), 0);
	glRotated(ghostsArray[i]->orientation, 0, 0, 1);
	glColor3f(1.0, 1.0, 1.0);
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texIds[ghostsArray[i]->getTexId()]);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex2f  (-0.5*ghostsArray[i]->width , -0.5*ghostsArray[i]->height);
	glTexCoord2f(0.0f,  1.0f);
	glVertex2f  (-0.5*ghostsArray[i]->width,  0.5*ghostsArray[i]->height);
	glTexCoord2f(1.0f,  1.0f);
	glVertex2f  (0.5*ghostsArray[i]->width,  0.5*ghostsArray[i]->height);
	glTexCoord2f(1.0f, 0.0f);
	glVertex2f  (0.5*ghostsArray[i]->width, -0.5*ghostsArray[i]->height);
	glEnd();
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();
	glColor3f(1.0, 1.0, 1.0);
    }
}

void GLWidget::drawCookies()
{
    for(int i = 0; i < sCookies; i++)
	drawCircle(cookiesCoord[i].x(), cookiesCoord[i].y(), 6.0, 1.0, 1.0, 0.0);
}

void GLWidget::drawBonus()
{
    for(int i = 0; i < sBonus; i++)
	drawCircle(bonusCoord[i].x(), bonusCoord[i].y(), 11.0, 1.0, 0.8, 0.0);
}

void GLWidget::drawCircle(float x, float y, float radius, float red, float green, float blue) 
{ 
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glTranslatef(x, y, 0.0f);
    glColor3f(red, green, blue);
    static const int circle_points = 100;
    static const float angle = 2.0f * 3.1416f / circle_points;

    // this code (mostly) copied from question:
    glBegin(GL_POLYGON);
    double angle1=0.0;
    glVertex2d(radius * cos(0.0) , radius * sin(0.0));
    int i;
    for (i=0; i<circle_points; i++)
    {       
        glVertex2d(radius * cos(angle1), radius *sin(angle1));
        angle1 += angle;
    }
    glEnd();
    glPopMatrix();
}

int GLWidget::getIndexRowFromCoord(QPoint coord)
{
    return (int)(abs(coord.y() - mapHeight*0.5));
}

int GLWidget::getIndexColFromCoord(QPoint coord)
{
    return (int)(coord.x() + mapWidth*0.5);
}

void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    loadTexture(new QImage(tr(":/resources/textures/pacman.jpeg")));    
    loadTexture(new QImage(tr(":/resources/textures/redGhostUp.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/redGhostDown.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/redGhostRight.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/redGhostLeft.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/pinkGhostUp.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/pinkGhostDown.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/pinkGhostRight.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/pinkGhostLeft.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/blueGhostUp.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/blueGhostDown.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/blueGhostRight.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/blueGhostLeft.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/orangeGhostUp.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/orangeGhostDown.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/orangeGhostRight.jpg")));
    loadTexture(new QImage(tr(":/resources/textures/orangeGhostLeft.jpg")));
    loadTexture(_mapImage);
}

void GLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(ortho[0], ortho[1], ortho[2], ortho[3], -10, 10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
      
    drawMap();
    drawPacman();
    drawGhosts();
    drawCookies();
    drawBonus();
}

void GLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(ortho[0], ortho[1], ortho[2], ortho[3], -10, 10);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void GLWidget::updateSimulationSlot()
{
    int i = 0;
    int stepX = pacmanArray[i]->width, stepY = pacmanArray[i]->height;
    if(pacmanArray[i]->action == Pacman::Action::Right)
    {
      QPoint coord1(pacmanArray[i]->currentPosition.x() + (int)(pacmanArray[i]->width*0.5)  + 1, pacmanArray[i]->currentPosition.y());
      if (obstacles[getIndexRowFromCoord(coord1)*mapWidth + getIndexColFromCoord(coord1)] != 1)
	  pacmanArray[i]->currentPosition.setX(pacmanArray[i]->currentPosition.x() + stepX);
      pacmanArray[i]->orientation = 0.0;
    }
    else if(pacmanArray[i]->action == Pacman::Action::Left)
    {
      QPoint coord1(pacmanArray[i]->currentPosition.x() - (int)(pacmanArray[i]->width*0.5) - 1, pacmanArray[i]->currentPosition.y());
      if (obstacles[getIndexRowFromCoord(coord1)*mapWidth + getIndexColFromCoord(coord1)] != 1)
	  pacmanArray[i]->currentPosition.setX(pacmanArray[i]->currentPosition.x() - stepX);
      pacmanArray[i]->orientation = 180.0;
    }
    else if(pacmanArray[i]->action == Pacman::Action::Up)
    {
      QPoint coord1(pacmanArray[i]->currentPosition.x(), pacmanArray[i]->currentPosition.y() + (int)(pacmanArray[i]->height*0.5) + 1);
      if (obstacles[getIndexRowFromCoord(coord1)*mapWidth + getIndexColFromCoord(coord1)] != 1)
	  pacmanArray[i]->currentPosition.setY(pacmanArray[i]->currentPosition.y() + stepY);
      pacmanArray[i]->orientation = 90.0;
    }
    else if(pacmanArray[i]->action == Pacman::Action::Down)
    {
      QPoint coord1(pacmanArray[i]->currentPosition.x(), pacmanArray[i]->currentPosition.y() - (int)(pacmanArray[i]->height*0.5) - 1);
      if (obstacles[getIndexRowFromCoord(coord1)*mapWidth + getIndexColFromCoord(coord1)] != 1)
	  pacmanArray[i]->currentPosition.setY(pacmanArray[i]->currentPosition.y() - stepY);
      pacmanArray[i]->orientation = 270.0;
    }
    pacmanArray[i]->action = Pacman::Action::None;
    update();//Schedule paintGL()
    emit UpdatePacmanPos(pacmanArray[i]->currentPosition);
  
}

void GLWidget::receiveMapDataGL(int blockWidth, int blockHeight, QImage* mapImage, int *mObstacles, QVector<int> *pPacman, QVector<int> *pGhosts, QVector<int> *pCookies, QVector<int> *pBonus)
{
    if(!firstTime)
      loadNewTexture(mapImage);
    else
      firstTime = false;

    //Map
    _mapImage = new QImage(*mapImage);
    mapWidth = mapImage->width();
    mapHeight = mapImage->height();
    obstacles = new int[(mapHeight)*(mapWidth)];
    memcpy(obstacles, mObstacles, (mapHeight)*(mapWidth)*sizeof(int));
    ortho[0] = -mapWidth*0.5;
    ortho[1] = mapWidth*0.5;
    ortho[2] = -mapHeight*0.5;
    ortho[3] = mapHeight*0.5;
        
    //Set Pacman
    nPacman = pPacman->size()/2;
    pacmanArray = (Pacman**) malloc(sizeof(Pacman)*nPacman);
    QPoint pacmanPositions;
    for(int i = 0; i < nPacman; i++)
    {
	pacmanPositions.setX(pPacman->at(i*2 + 1)*blockWidth+ortho[0]+blockWidth*0.5);
	pacmanPositions.setY(ortho[3]-(pPacman->at(i*2))*blockHeight-blockHeight*0.5);
	pacmanArray[i] = new Pacman(pacmanPositions, (double)0, blockHeight, blockWidth);
    }
    
    //cout << "nPacman = " << nPacman << endl;
        
        
    //Set Ghosts
    nGhosts = pGhosts->size()/2;
    ghostsArray = (Ghosts**) malloc(sizeof(Ghosts)*nGhosts);
    QPoint ghostPositions;
    Ghosts::Personality ghostsPersonality[4] = {Ghosts::Personality::Shadow, Ghosts::Personality::Speedy, Ghosts::Personality::Bashful, Ghosts::Personality::Pokey};
    for(int i = 0; i < nGhosts; i++)
    {
	ghostPositions.setX(pGhosts->at(i*2 + 1)*blockWidth+ortho[0]+blockWidth*0.5);
	ghostPositions.setY(ortho[3]-(pGhosts->at(i*2))*blockHeight-blockHeight*0.5);
	ghostsArray[i] = new Ghosts(ghostPositions, ghostsPersonality[i], blockHeight, blockWidth);
    }
    
    //Set cookies
    sCookies = pCookies->size()/2;
    cookiesCoord = new QPoint[sCookies];
    for(int i = 0; i < sCookies; i++)
    {
	cookiesCoord[i].setX(pCookies->at(i*2 + 1)*blockWidth+ortho[0]+blockWidth*0.5);
	cookiesCoord[i].setY(ortho[3]-(pCookies->at(i*2))*blockHeight-blockHeight*0.5);
    }
    
    //Set bonuses
    sBonus = pBonus->size()/2;
    bonusCoord = new QPoint[sBonus];
    for(int i = 0; i < sBonus; i++)
    {
	bonusCoord[i].setX(pBonus->at(i*2 + 1)*blockWidth+ortho[0]+blockWidth*0.5);
	bonusCoord[i].setY(ortho[3]-(pBonus->at(i*2))*blockHeight-blockHeight*0.5);
    }    
    update();
}

void GLWidget::setPacmanCommand(int aPacmanCommand)
{
    pacmanArray[0]->action = static_cast<Pacman::Action>(aPacmanCommand);
}