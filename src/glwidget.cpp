#include "pacman/glwidget.h"

GLWidget::GLWidget(QWidget *parent)
{
    mapImage = new QImage(tr(":/resources/textures/bigSafeSearch.png"));
    pacmanImage = new QImage(tr(":/resources/textures/pacman.jpeg"));
    mapWidth = mapImage->width();
    mapHeight = mapImage->height();
    pacmanHeight = 20;
    pacmanWidth = 20;
    memset(obstacles, 1, 1000*1000*sizeof(bool));
    QVector<QRgb> table = mapImage->colorTable();
    cout << "width = " << mapImage->width() << ", length = " << mapImage->height() << " format = " << mapImage->format() << ", Colors = " << table.size() << endl;
    cout << "table: " << endl;
    for(int i = 0;i < table.size();i++)
    {
	cout << qRed(table[i]) << ", " << qGreen(table[i]) << ", " << qBlue(table[i]) << ", " << table[i] << endl;
    }
    
    for(int i = 0;i < mapImage->height(); i++)
    {
	for (int j = 0;j < mapImage->width();j++)
	{
	    QColor clrCurrent( mapImage->pixel(j, i) );
	    if(clrCurrent.red() == 255 && clrCurrent.green() == 255 && clrCurrent.blue() == 255)
	    {
		obstacles[i][j] = true;
	    }
	    else
	    {
		obstacles[i][j] = false;
		//mapImage->setPixel(j, i, qRgba(0, 0, 0, 255));
	    }
	}
    }
   
    //Map coordinates
    ortho[0] = -mapWidth*0.5;
    ortho[1] = mapWidth*0.5;
    ortho[2] = -mapHeight*0.5;
    ortho[3] = mapHeight*0.5;
    
    //Pacman's initial pose
    pacmanCoord.setX(-14*pacmanWidth);
    pacmanCoord.setY(-2*pacmanHeight);
    w = 0.0; 
}

GLWidget::~GLWidget()
{
    cleanup();
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(mapWidth, mapHeight);
}

void GLWidget::cleanup()
{
    if (m_program == nullptr)
        return;
    makeCurrent();
    m_logoVbo.destroy();
    delete m_program;
    m_program = 0;
    doneCurrent();
}

static const char *vertexShaderSourceCore =
    "#version 150\n"
    "in vec4 vertex;\n"
    "in vec3 normal;\n"
    "out vec3 vert;\n"
    "out vec3 vertNormal;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertNormal = normalMatrix * normal;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSourceCore =
    "#version 150\n"
    "in highp vec3 vert;\n"
    "in highp vec3 vertNormal;\n"
    "out highp vec4 fragColor;\n"
    "uniform highp vec3 lightPos;\n"
    "void main() {\n"
    "   highp vec3 L = normalize(lightPos - vert);\n"
    "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
    "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   fragColor = vec4(col, 1.0);\n"
    "}\n";

static const char *vertexShaderSource =
    "attribute vec4 vertex;\n"
    "attribute vec3 normal;\n"
    "varying vec3 vert;\n"
    "varying vec3 vertNormal;\n"
    "uniform mat4 projMatrix;\n"
    "uniform mat4 mvMatrix;\n"
    "uniform mat3 normalMatrix;\n"
    "void main() {\n"
    "   vert = vertex.xyz;\n"
    "   vertNormal = normalMatrix * normal;\n"
    "   gl_Position = projMatrix * mvMatrix * vertex;\n"
    "}\n";

static const char *fragmentShaderSource =
    "varying highp vec3 vert;\n"
    "varying highp vec3 vertNormal;\n"
    "uniform highp vec3 lightPos;\n"
    "void main() {\n"
    "   highp vec3 L = normalize(lightPos - vert);\n"
    "   highp float NL = max(dot(normalize(vertNormal), L), 0.0);\n"
    "   highp vec3 color = vec3(0.39, 1.0, 0.0);\n"
    "   highp vec3 col = clamp(color * 0.2 + color * 0.8 * NL, 0.0, 1.0);\n"
    "   gl_FragColor = vec4(col, 1.0);\n"
    "}\n";

void GLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    loadTexture(mapImage);
    loadTexture(pacmanImage);
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
    glBindTexture(GL_TEXTURE_2D, texIds[0]);
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
    glBindTexture(GL_TEXTURE_2D, texIds[1]);
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

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    m_lastPos = event->pos();
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - m_lastPos.x();
    int dy = event->y() - m_lastPos.y();
    m_lastPos = event->pos();
}

void GLWidget::receiveKeySlot(int key)
{
    int threshold = 0;
    int stepX = pacmanWidth, stepY = pacmanHeight;
    if(key == Qt::Key_D)
    {
      QPoint coord1(pacmanCoord.x() + (int)(pacmanWidth*0.5) + threshold + 1, pacmanCoord.y());
      if (!obstacles[getIndexRowFromCoord(coord1)][getIndexColFromCoord(coord1)])
	  pacmanCoord.setX(pacmanCoord.x() + stepX);
      w = 0.0;
      update();
    }
    else if(key == Qt::Key_A)
    {
      QPoint coord1(pacmanCoord.x() - (int)(pacmanWidth*0.5) - threshold - 1, pacmanCoord.y());
      if (!obstacles[getIndexRowFromCoord(coord1)][getIndexColFromCoord(coord1)])
	  pacmanCoord.setX(pacmanCoord.x() - stepX);
      w = 180.0;
      update();
    }
    else if(key == Qt::Key_W)
    {
      QPoint coord1(pacmanCoord.x(), pacmanCoord.y() + (int)(pacmanHeight*0.5) + threshold + 1);
      if (!obstacles[getIndexRowFromCoord(coord1)][getIndexColFromCoord(coord1)])
	  pacmanCoord.setY(pacmanCoord.y() + stepY);
      w = 90.0;
      update();
    }
    else if(key == Qt::Key_S)
    {
      QPoint coord1(pacmanCoord.x(), pacmanCoord.y() - (int)(pacmanHeight*0.5) - threshold - 1);
      if (!obstacles[getIndexRowFromCoord(coord1)][getIndexColFromCoord(coord1)])
	  pacmanCoord.setY(pacmanCoord.y() - stepY);
      w = 270.0;
      update();
    }
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

int GLWidget::getIndexRowFromCoord(QPoint coord)
{
    return (int)(abs(coord.y() - mapHeight*0.5));
}

int GLWidget::getIndexColFromCoord(QPoint coord)
{
    return (int)(coord.x() + mapWidth*0.5);
}
