// Self include
#include "face.h"

Face::Face(const Vector& _localUp)
{
	localUp = _localUp;
	axisA = Vector(localUp[1], localUp[2], localUp[0]);
	axisB = localUp / axisA;
}

Vector Face::getLocalUp() const
{
	return localUp;
}

Vector Face::getAxisA() const
{
	return axisA;
}

Vector Face::getAxisB() const
{
	return axisB;
}

