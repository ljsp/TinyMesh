#pragma once

// Terrain 

#include <vector>
#include <iostream>
#include "mathematics.h"

class Terrain
{
public:
	Terrain(const Vector&);
	Vector getLocalUp() const;
	Vector getAxisA() const;
	Vector getAxisB() const;
	~Terrain() {}
private:
	Vector localUp;
	Vector axisA;
	Vector axisB;
};
	
