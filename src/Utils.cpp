#include "pacman/Utilities.h"

Utilities::Utilities()
{

}

int Utilities::getIndexColFromCoord(QPoint coord, int mapWidth)
{
    return (int)(coord.x() + mapWidth*0.5);
}

int Utilities::getIndexRowFromCoord(QPoint coord, int mapHeight)
{
    return (int)(abs(coord.y() - mapHeight*0.5));
}