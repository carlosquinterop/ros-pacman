#include "pacman/Utilities.h"

Utilities::Utilities()
{

}

void Utilities::SetMapData(int blockWidth, int blockHeight, double ortho[4])
{
    _blockWidth = blockWidth;
    _blockHeight = blockHeight;
    for(int i = 0; i < 4; i++)
	_ortho[i] = ortho[i];
}

int Utilities::GetIndexColFromCoord(QPoint coord, int mapWidth)
{
    return (int)(coord.x() + mapWidth*0.5);
}

int Utilities::GetIndexRowFromCoord(QPoint coord, int mapHeight)
{
    return (int)(abs(coord.y() - mapHeight*0.5));
}

QPoint* Utilities::GetCoordFromIndex(int iRow, int iCol)
{
    QPoint *p = new QPoint;
    p->setX(iCol*_blockWidth+_ortho[0]+_blockWidth*0.5);
    p->setY(_ortho[3]-iRow*_blockHeight-_blockHeight*0.5);
    
    return p;
}

int Utilities::ComputeDistanceBetweenPoints(QPoint a, QPoint b)
{
    return (int)sqrt((a.x() - b.x())*(a.x() - b.x()) + (a.y() - b.y())*(a.y() - b.y()));
}