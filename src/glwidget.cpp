#include "pacman/glwidget.h"

GLWidget::GLWidget(QWidget *parent)
{
	firstTime = true;
	pacmanImage = new QImage(tr(":/resources/textures/pacman.jpeg"));
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

void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    loadTexture(pacmanImage);
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
      
    // Dibujando mapa
    glColor3f(0.7, 0.7, 0.7);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texIds[1]);
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
       
    //Dibujando pacman
    glShadeModel(GL_SMOOTH);
    glPushMatrix();
    glTranslated(pacmanCoord.x(), pacmanCoord.y(), 0);
    glRotated(w, 0, 0, 1);
    glColor3f(1.0, 1.0, 1.0);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texIds[0]);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f  (-0.5*pacmanWidth, -0.5*pacmanHeight);
    glTexCoord2f(0.0f,  1.0f);
    glVertex2f  (-0.5*pacmanWidth,  0.5*pacmanHeight);
    glTexCoord2f(1.0f,  1.0f);
    glVertex2f  (0.5*pacmanWidth,  0.5*pacmanHeight);
    glTexCoord2f(1.0f, 0.0f);
    glVertex2f  (0.5*pacmanWidth, -0.5*pacmanHeight);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    glPopMatrix();
    glColor3f(1.0, 1.0, 1.0);
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

void GLWidget::receiveKeySlot(int key)
{
    int threshold = 0;
    int stepX = pacmanWidth, stepY = pacmanHeight;
    if(key == Qt::Key_D)
    {
      QPoint coord1(pacmanCoord.x() + (int)(pacmanWidth*0.5) + threshold + 1, pacmanCoord.y());
      if (!obstacles[getIndexRowFromCoord(coord1)*mapWidth + getIndexColFromCoord(coord1)])
	  pacmanCoord.setX(pacmanCoord.x() + stepX);
      w = 0.0;
      update();
    }
    else if(key == Qt::Key_A)
    {
      QPoint coord1(pacmanCoord.x() - (int)(pacmanWidth*0.5) - threshold - 1, pacmanCoord.y());
      if (!obstacles[getIndexRowFromCoord(coord1)*mapWidth + getIndexColFromCoord(coord1)])
	  pacmanCoord.setX(pacmanCoord.x() - stepX);
      w = 180.0;
      update();
    }
    else if(key == Qt::Key_W)
    {
      QPoint coord1(pacmanCoord.x(), pacmanCoord.y() + (int)(pacmanHeight*0.5) + threshold + 1);
      if (!obstacles[getIndexRowFromCoord(coord1)*mapWidth + getIndexColFromCoord(coord1)])
	  pacmanCoord.setY(pacmanCoord.y() + stepY);
      w = 90.0;
      update();
    }
    else if(key == Qt::Key_S)
    {
      QPoint coord1(pacmanCoord.x(), pacmanCoord.y() - (int)(pacmanHeight*0.5) - threshold - 1);
      if (!obstacles[getIndexRowFromCoord(coord1)*mapWidth + getIndexColFromCoord(coord1)])
	  pacmanCoord.setY(pacmanCoord.y() - stepY);
      w = 270.0;
      update();
    }
}

void GLWidget::receiveMapDataGL(int wPacman, int hPacman, QImage* mapImage, bool *mObstacles, int rowPacman, int colPacman)
{
	if(!firstTime)
		loadNewTexture(mapImage);
	else
		firstTime = false;
		
	_mapImage = new QImage(*mapImage);	
	mapWidth = mapImage->width();
    mapHeight = mapImage->height();
    pacmanHeight = hPacman;
    pacmanWidth = wPacman;
    obstacles = new bool[(mapHeight)*(mapWidth)];
    memcpy(obstacles, mObstacles, (mapHeight)*(mapWidth)*sizeof(bool));
    //Map coordinates
    ortho[0] = -mapWidth*0.5;
    ortho[1] = mapWidth*0.5;
    ortho[2] = -mapHeight*0.5;
    ortho[3] = mapHeight*0.5;
    //Pacman's initial pose
    pacmanCoord.setX(colPacman*pacmanWidth+ortho[0]+pacmanWidth*0.5);
    pacmanCoord.setY(ortho[3]-rowPacman*pacmanHeight-pacmanHeight*0.5);
    w = 0.0; 
    update();
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
    texIds[1] = tex;	
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, t.width(), t.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, t.bits());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glDisable(GL_TEXTURE_2D);
}

int GLWidget::getIndexRowFromCoord(QPoint coord)
{
    return (int)(abs(coord.y() - mapHeight*0.5));
}

int GLWidget::getIndexColFromCoord(QPoint coord)
{
    return (int)(coord.x() + mapWidth*0.5);
}
