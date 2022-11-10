#pragma once

// Terrain 

#include <vector>
#include <iostream>
#include "mathematics.h"

class Face
{
public:
	Face(const Vector&);
	Vector getLocalUp() const;
	Vector getAxisA() const;
	Vector getAxisB() const;
	~Face() {}
private:
	Vector localUp;
	Vector axisA;
	Vector axisB;
};
	
