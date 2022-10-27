// Self include
#include "terrain.h"

Terrain::Terrain(const Vector& _localUp)
{
	localUp = _localUp;
	axisA = Vector(localUp[1], localUp[2], localUp[0]);
	axisB = localUp / axisA;
}

Vector Terrain::getLocalUp() const
{
	return localUp;
}

Vector Terrain::getAxisA() const
{
	return axisA;
}

Vector Terrain::getAxisB() const
{
	return axisB;
}

