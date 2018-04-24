#pragma once
class Joint
{
public:
	
	int prevXpos, prevYpos;
	
	Joint();
	~Joint();


	void setX(int x) { xPos = x; }
	void setY(int y) { yPos = y; }
	int getX() { return xPos; }
	int getY() { return yPos; }



private:

	int xPos, yPos;
	
};

