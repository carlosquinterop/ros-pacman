#include "pacman/Ghosts.h"

Ghosts::Ghosts(QPoint initialPosition, Ghosts::Personality aCharacter, int aHeight, int aWidth, QPoint initialPacmanPosition, int cMapHeight, int cMapWidth, int *cObstacles)
{
    mode = Mode::Scatter;
    currentPosition = initialPosition;
    character = aCharacter;
    orientation = 180.0;
    height = aHeight;
    width = aWidth;
    action = Action::None;
    pacmanPosition = initialPacmanPosition;
    mapHeight = cMapHeight;
    mapWidth = cMapWidth;
    obstacles = new int[(mapHeight)*(mapWidth)];
    memcpy(obstacles, cObstacles, (mapHeight)*(mapWidth)*sizeof(int));
        
    if (character == Ghosts::Personality::Shadow)
	name = "Blinky";
    else if (character == Ghosts::Personality::Speedy)
	name = "Pinky";
    else if (character == Ghosts::Personality::Bashful)
	name = "Inky";
    else if (character == Ghosts::Personality::Pokey)
	name = "Clyde";
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

void Ghosts::updateGhostPosition()
{
    int stepX = width;
    int stepY = height;
    if (action == Ghosts::Action::None)
    {
	QPoint coordLeft(currentPosition.x() - stepX, currentPosition.y());
	QPoint coordRight(currentPosition.x() + stepX, currentPosition.y());
	QPoint coordUp(currentPosition.x(), currentPosition.y() + stepY);
	QPoint coordDown(currentPosition.x(), currentPosition.y() - stepY);
	
	if (obstacles[utilities.getIndexRowFromCoord(coordLeft, mapHeight)*mapWidth + utilities.getIndexColFromCoord(coordLeft, mapWidth)] != 1)
	    action = Ghosts::Action::Left;
	else if (obstacles[utilities.getIndexRowFromCoord(coordDown, mapHeight)*mapWidth + utilities.getIndexColFromCoord(coordDown, mapWidth)] != 1)
	    action = Ghosts::Action::Down;
	else if (obstacles[utilities.getIndexRowFromCoord(coordRight, mapHeight)*mapWidth + utilities.getIndexColFromCoord(coordRight, mapWidth)] != 1)
	    action = Ghosts::Action::Right;
	else if (obstacles[utilities.getIndexRowFromCoord(coordUp, mapHeight)*mapWidth + utilities.getIndexColFromCoord(coordUp, mapWidth)] != 1)
	    action = Ghosts::Action::Up;
    }
    else
    {
	QVector<Ghosts::Action> *possibleActions;
	possibleActions = new QVector<Ghosts::Action>;
	getPossibleActions(possibleActions);

	if (possibleActions->size() == 1)
	    action = possibleActions->at(0);
	else if (possibleActions->size() == 2)
	{
	    if (!isPossibleAction(action, possibleActions))
		action = possibleActions->at(rand() % possibleActions->size()); 
	}
	else if (possibleActions->size() >= 3)
	{
	    calculateTargetPosition();
	    action = possibleActions->at(rand() % possibleActions->size());
	}
    }
    
    if (action == Ghosts::Action::Left)
    {
	currentPosition.setX(currentPosition.x() - stepX);
	orientation = 180.0;
    }
    else if (action == Ghosts::Action::Right)
    {
	currentPosition.setX(currentPosition.x() + stepX);
	orientation = 0.0;
    }
    else if (action == Ghosts::Action::Up)
    {
	currentPosition.setY(currentPosition.y() + stepY);
	orientation = 90;
    }
    else if (action == Ghosts::Action::Down)
    {
	currentPosition.setY(currentPosition.y() - stepY);
	orientation = 270;
    }
}

void Ghosts::calculateTargetPosition()
{
    /*
    if (character == Ghosts::Personality::Shadow)
    {
	targetPosition
    }
    else if (character == Ghosts::Personality::Speedy)
    {
	targetPosition
    }
    else if (character == Ghosts::Personality::Bashful)
    {
	targetPosition
    }
    else if (character == Ghosts::Personality::Pokey)
    {
	targetPosition
    }*/
}

int Ghosts::getPossibleActions(QVector<Ghosts::Action>* possibleActions)
{
    QPoint coordLeft(currentPosition.x() - (int)(width*0.5) - 1, currentPosition.y());
    QPoint coordDown(currentPosition.x(), currentPosition.y() - (int)(height*0.5) - 1);
    QPoint coordRight(currentPosition.x() + (int)(width*0.5)  + 1, currentPosition.y());
    QPoint coordUp(currentPosition.x(), currentPosition.y() + (int)(height*0.5) + 1);

    if (obstacles[utilities.getIndexRowFromCoord(coordLeft, mapHeight)*mapWidth + utilities.getIndexColFromCoord(coordLeft, mapWidth)] != 1)
	possibleActions->append(Ghosts::Action::Left);
    if (obstacles[utilities.getIndexRowFromCoord(coordDown, mapHeight)*mapWidth + utilities.getIndexColFromCoord(coordDown, mapWidth)] != 1)
	possibleActions->append(Ghosts::Action::Down);
    if (obstacles[utilities.getIndexRowFromCoord(coordRight, mapHeight)*mapWidth + utilities.getIndexColFromCoord(coordRight, mapWidth)] != 1)
	possibleActions->append(Ghosts::Action::Right);
    if (obstacles[utilities.getIndexRowFromCoord(coordUp, mapHeight)*mapWidth + utilities.getIndexColFromCoord(coordUp, mapWidth)] != 1)
	possibleActions->append(Ghosts::Action::Up);
}

void Ghosts::printPossibleActions(QVector<Ghosts::Action>* possibleActions)
{
    for (int i = 0;i < possibleActions->size();i++)
    {
	if(possibleActions->at(i) == Ghosts::Action::Down)
	    cout << "   Down at " << i << endl;
	if(possibleActions->at(i) == Ghosts::Action::Left)
	    cout << "   Left " << i << endl;
	if(possibleActions->at(i) == Ghosts::Action::Up)
	    cout << "   Up " << i << endl;
	if(possibleActions->at(i) == Ghosts::Action::Right)
	    cout << "   Right " << i << endl;
    }
}

bool Ghosts::isPossibleAction(Ghosts::Action anAction, QVector<Ghosts::Action>* possibleActions)
{
    bool isPossible = false;
    for (int i = 0;i < possibleActions->size();i++)
    {
	if (possibleActions->at(i) == anAction)
	    isPossible |= true;
    }
    return isPossible;
}