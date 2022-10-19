#pragma once


#include <math.h>
#include <ostream>

class Matrix3
{
public:
	Matrix3();
	Matrix3 Scale(const double);
	Matrix3 RotateX(const double);
	Matrix3 RotateY(const double);
	Matrix3 RotateZ(const double);
	Matrix3 Rotate(const double, const double, const double);

private:
	double matrix[9];
};