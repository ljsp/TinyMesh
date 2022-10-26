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
	void Transpose();
	void Inverse();
	void Scale(const double);
	void RotateX(double);
	void RotateY(double);
	void RotateZ(double);
	void Rotate(const Vector, const double);
private:
	std::array<double,9> matrix;
};
