#pragma once


#include <mathematics.h>
#include <array>
#include <ostream>

class Matrix3
{
public:
	Matrix3();
	Matrix3(
		const double, const double, const double,
		const double, const double, const double,
		const double, const double, const double);
	std::array<double, 9> GetMatrix() const;
	Matrix3 operator*(const Matrix3& m) const;
	Vector operator*(const Vector& v) const;
	void Identity();
	Matrix3 Transpose();
	Matrix3 Inverse();
	Matrix3 Scale(const double);
	Matrix3 RotateX(double);
	Matrix3 RotateY(double);
	Matrix3 RotateZ(double);
	Matrix3 Rotate(const double, const double, const double);
	Matrix3 Rotate(const Vector, const double);
private:
	std::array<double,9> matrix;
};
