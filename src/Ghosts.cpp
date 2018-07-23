#include "pacman/Ghosts.h"

Ghosts::Ghosts(QPoint initialPosition, Ghosts::Personality aCharacter, int aHeight, int aWidth)
{
    mode = Mode::Scatter;
    currentPosition = initialPosition;
    character = aCharacter;
    orientation = 0;
    height = aHeight;
    width = aWidth;
    previousAction = Action::None;
    action = Action::None;
    if (character == Ghosts::Personality::Shadow)
    {
	name = "Blinky";
	upFileName = tr(":/resources/textures/redGhostUp.jpg");
	downFileName = tr(":/resources/textures/redGhostDown.jpg");
	rightFileName = tr(":/resources/textures/redGhostRight.jpg");
	leftFileName = tr(":/resources/textures/redGhostLeft.jpg");
	//targetPosition
    }
    else if (character == Ghosts::Personality::Speedy)
    {
	name = "Pinky";
	upFileName = tr(":/resources/textures/pinkGhostUp.jpg");
	downFileName = tr(":/resources/textures/pinkGhostDown.jpg");
	rightFileName = tr(":/resources/textures/pinkGhostRight.jpg");
	leftFileName = tr(":/resources/textures/pinkGhostLeft.jpg");
	//targetPosition
    }
    else if (character == Ghosts::Personality::Bashful)
    {
	name = "Inky";
	upFileName = tr(":/resources/textures/blueGhostUp.jpg");
	downFileName = tr(":/resources/textures/blueGhostDown.jpg");
	rightFileName = tr(":/resources/textures/blueGhostRight.jpg");
	leftFileName = tr(":/resources/textures/blueGhostLeft.jpg");
	//targetPosition
    }
    else if (character == Ghosts::Personality::Pokey)
    {
	name = "Clyde";
	upFileName = tr(":/resources/textures/orangeGhostUp.jpg");
	downFileName = tr(":/resources/textures/orangeGhostDown.jpg");
	rightFileName = tr(":/resources/textures/orangeGhostRight.jpg");
	leftFileName = tr(":/resources/textures/orangeGhostLeft.jpg");
	//targetPosition
    }
    getTexId();
}

int Ghosts::getTexId()
{
    if (character == Ghosts::Personality::Shadow)
    { 
	if (orientation == 90)
	    texId = 1;
	else if (orientation == 270)
	    texId = 2;
	else if (orientation == 0)
	    texId = 3;
	else if (orientation == 180)
	    texId = 4;
    }
    else if (character == Ghosts::Personality::Speedy)
    { 
	if (orientation == 90)
	    texId = 5;
	else if (orientation == 270)
	    texId = 6;
	else if (orientation == 0)
	    texId = 7;
	else if (orientation == 180)
	    texId = 8;
    }
    else if (character == Ghosts::Personality::Bashful)
    { 
	if (orientation == 90)
	    texId = 9;
	else if (orientation == 270)
	    texId = 10;
	else if (orientation == 0)
	    texId = 11;
	else if (orientation == 180)
	    texId = 12;
    }
    else if (character == Ghosts::Personality::Pokey)
    { 
	if (orientation == 90)
	    texId = 13;
	else if (orientation == 270)
	    texId = 14;
	else if (orientation == 0)
	    texId = 15;
	else if (orientation == 180)
	    texId = 16;
    }
    else
	texId = -1;
    
    return texId;
}
