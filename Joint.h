#pragma once
class Joint
{
public:
	
	int prevXpos = 0, prevYpos = 0;
	int identifier;
	
	Joint();
	~Joint();


	void setX(int x) { xPos = x; }
	void setY(int y) { yPos = y; }
	int getX() { return xPos; }
	int getY() { return yPos; }



private:

	int xPos, yPos;
	
};

