#include "pacman/maps.h"

Maps::Maps(QWidget* parent)
{
    
}

QByteArray Maps::file2ArrayMap(QString fileName, int& colsText, int& rowsText)
{
    //Carga archivo .lay
    QFile file(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/layouts/" + fileName);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return nullptr;
    
    //Lectura datos mapa
    QByteArray text;
    QByteArray line;
    bool firstLine = false;
    while (!file.atEnd()) {
        line = file.readLine();
	for(int i = 0; i < line.size(); i++)
	{
	    if(line[i] != '\n')
		text.append(line[i]);
	}
	
	if(!firstLine)
	{
	    colsText = line.size() - 1;
	    firstLine = true;
	}
    }
    rowsText = (text.size()/(colsText));
 	
 	//Hacer que el mapa tenga siempre un centro cuando se convierte a coordenadas
    if(rowsText%2 == 0)
    {
		for(int i = 0; i < colsText; i++)
		    text.append('%');
		rowsText++;
    }
    if(colsText%2 == 0)
    {
		for(int i = 0; i < rowsText; i++)
		    text.insert( (i+1)*colsText + i, '%');
		colsText++;
    }	
 	   
    return text;
}

void Maps::createObstaclesArray(QByteArray text, int colsText, int rowsText)
{
    cols = colsText*BLOCK_SIZE;
    rows = rowsText*BLOCK_SIZE;
    mObstacles = new bool[(rows)*(cols)];
    
    //Inicializacion matriz obstaculos
    for(int i = 0; i < rows; i++)
	for(int j = 0; j < cols; j++)
	    mObstacles[i*cols + j] = false;
    
    //Construccion matriz de obstaculos
    for(int i = 0; i < rowsText; i++)
    {
	for(int j = 0; j < colsText; j++)
	{
	    if(text[i*colsText + j] == '%')
	    {
		for(int k = (i*BLOCK_SIZE); k < ((i*BLOCK_SIZE) + BLOCK_SIZE); k++)
		{
		    for(int l = (j*BLOCK_SIZE); l < ((j*BLOCK_SIZE) + BLOCK_SIZE); l++)
		    {
			mObstacles[k*cols + l] = true;
		    }
		}
	    }
	    if(text[i*colsText + j] == 'P')
	    {
		rowPacman = i;
		colPacman = j;		
	    } 
	}
    }
}

void Maps::printObstaclesArray()
{
    //Imprimir matriz obstaculos
    for(int i = 0; i < rows; i++)
    {
	for(int j = 0; j < cols; j++)
	    cout << mObstacles[i*cols + j] << "";
	cout << endl;
    }
}

void Maps::createImageFromObstaclesArray()
{
    image = new QImage(cols, rows, QImage::Format_RGB32);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            QRgb value;
            if(mObstacles[i*cols + j] == true)
            {
                value = qRgb(255,255,255);
		image->setPixel(j, i, value);
            }
            else
            {
		value = qRgb(0,0,0);
		image->setPixel(j, i, value);
            }
      }
   }
}

void Maps::saveImage(QString fileName)
{
    //Guardar Imagen
   QFile fileMap(QString::fromStdString(ros::package::getPath("pacman")) + "/resources/textures/" + fileName);
   if(!fileMap.exists())//Si existe la imagen no se guarda
   {
      if(fileMap.open(QIODevice::WriteOnly | QIODevice::Truncate))
	  image->save(&fileMap, "PNG");
      else
	  cout << "No se pudo abrir" << endl;
   }
}
void Maps::createMap(QString nameMap)
{
    rowPacman = -1;
    colPacman = -1;
    
    //Lectura de archivo
    int colsText, rowsText;
    QByteArray text = file2ArrayMap(nameMap, colsText, rowsText);
    
    //Creacion matriz de obstaculos
    createObstaclesArray(text, colsText, rowsText);
    
    //Impresion matriz obstaculos
    //printObstaclesArray();
    
    //Creacion imagen
    createImageFromObstaclesArray();

    //Guardar imagen
    saveImage(nameMap.split(".").at(0) + ".png");
    
    emit sendMapData(BLOCK_SIZE, BLOCK_SIZE, image, mObstacles, rowPacman, colPacman);
}

