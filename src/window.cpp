#include "pacman/window.h"


Window::Window()
{
    maps = new Maps;
	glWidget = new GLWidget;
	allowPlay = false;
	
    QVBoxLayout *mainLayout = new QVBoxLayout;
    container = new QHBoxLayout;
       
    QWidget *w = new QWidget;
    w->setLayout(container);
    mainLayout->addWidget(w);
    playBtn = new QPushButton(tr("Play"));
    connect(playBtn, &QPushButton::clicked, this, &Window::playSlot);
    mainLayout->addWidget(playBtn);
    
    mapsList = new QComboBox;    
    listArrayMap(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/layouts/");
    
    connect(mapsList, SIGNAL(currentIndexChanged(QString)), maps, SLOT(createMap(QString)));
    connect(maps, SIGNAL(sendMapData(int,int,QImage*,bool*,int,int)), glWidget, SLOT(receiveMapDataGL(int,int,QImage*,bool*,int,int)));
	connect(this, SIGNAL(arrowKey(int)), glWidget, SLOT(receiveKeySlot(int)));
	
	container->addWidget(glWidget);
    maps->createMap(mapsList->currentText());
    mainLayout->addWidget(mapsList);

    setLayout(mainLayout);
}

void Window::keyPressEvent(QKeyEvent *e)
{
	if(allowPlay)
   		emit arrowKey(e->key());
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
		allowPlay = true;
		mapsList->setEnabled(false);
	}
	else
	{
		playBtn->setText("Play");
		allowPlay = false;
		maps->createMap(mapsList->currentText());
		mapsList->setEnabled(true);
	}
	
}
