#include "pacman/Pacman.h"

Pacman::Pacman(QPoint initialPosition, double initialOrientation, int aHeight, int aWidth)
{
    currentPosition = initialPosition;
    orientation = initialOrientation;
    height = aHeight;
    width = aWidth;
    fileName = tr(":/resources/textures/pacman.jpeg");
    action = Pacman::Action::None;
}