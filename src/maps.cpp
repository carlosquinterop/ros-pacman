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
    mObstacles = new int[(rows)*(cols)];
    
    //Inicializacion matriz obstaculos
    for(int i = 0; i < rows; i++)
	for(int j = 0; j < cols; j++)
	    mObstacles[i*cols + j] = 0;
    
    pPacman = new QVector<int>;
    pGhosts = new QVector<int>;
    pCookies = new QVector<int>;
    pBonus = new QVector<int>;
    
    //Construccion matriz de obstaculos
    for(int i = 0; i < rowsText; i++)
    {
	for(int j = 0; j < colsText; j++)
	{
	    if(text[i*colsText + j] == '%' || (text[i*colsText + j] == '.') || (text[i*colsText + j] == 'o'))
	    {
		int aux = 0;
		if(text[i*colsText + j] == '%') //Obstaculos
		{
		    aux = 1;
		}
		else if(text[i*colsText + j] == '.') //Galletas
		{
		    pCookies->append(i);
		    pCookies->append(j);
		    aux = 2;
		}
		else //Bonos
		{
		    pBonus->append(i);
		    pBonus->append(j);
		    aux = 3;
		}
		
		for(int k = (i*BLOCK_SIZE); k < ((i*BLOCK_SIZE) + BLOCK_SIZE); k++)
		{
		    for(int l = (j*BLOCK_SIZE); l < ((j*BLOCK_SIZE) + BLOCK_SIZE); l++)
		    {
			mObstacles[k*cols + l] = aux;
		    }
		}
	    }
	    	    	    
	    //Pacman
	    if(text[i*colsText + j] == 'P')
	    {
		pPacman->append(i);
		pPacman->append(j);	
	    } 
	    
	    //Ghosts
	    if(text[i*colsText + j] == 'G')
	    {
		pGhosts->append(i);
		pGhosts->append(j);
	    } 
	}
    }
}

void Maps::printObstaclesArray()
{
    //Imprimir matriz obstaculos
    cout << "------------------------------------------" <<endl;
    for(int i = 0; i < rows; i++)
    {
	for(int j = 0; j < cols; j++)
	    cout << mObstacles[i*cols + j] << "";
	cout << endl;
    }
    cout << "------------------------------------------" <<endl;
}

void Maps::createImageFromObstaclesArray()
{
    image = new QImage(cols, rows, QImage::Format_RGB32);
    for(int i = 0; i < rows; i++)
    {
        for(int j = 0; j < cols; j++)
        {
            QRgb value;
            if(mObstacles[i*cols + j] == 1)
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
    //Lectura de archivo
    int colsText, rowsText;
    QByteArray text = file2ArrayMap(nameMap + ".lay", colsText, rowsText);
    
    //Creacion matriz de obstaculos
    createObstaclesArray(text, colsText, rowsText);
    
    //Impresion matriz obstaculos
    //printObstaclesArray();
    
    //Creacion imagen
    createImageFromObstaclesArray();

    //Guardar imagen
    saveImage(nameMap + ".png");
    
    emit sendMapData(BLOCK_SIZE, BLOCK_SIZE, image, mObstacles, pPacman, pGhosts, pCookies, pBonus);
}

