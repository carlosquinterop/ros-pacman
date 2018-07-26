#include "pacman/Utilities.h"

Utilities::Utilities()
{

}

int Utilities::GetIndexColFromCoord(QPoint coord, int mapWidth)
{
    return (int)(coord.x() + mapWidth*0.5);
}

int Utilities::GetIndexRowFromCoord(QPoint coord, int mapHeight)
{
    return (int)(abs(coord.y() - mapHeight*0.5));
}

int Utilities::ComputeDistanceBetweenPoints(QPoint a, QPoint b)
{
    return (int)sqrt((a.x() - b.x())*(a.x() - b.x()) + (a.y() - b.y())*(a.y() - b.y()));
}