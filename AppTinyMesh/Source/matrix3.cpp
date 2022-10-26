#include "matrix3.h"
#include <mathematics.h>

// Matrix

Matrix3::Matrix3(
	const double m0, const double m1, const double m2,
	const double m3, const double m4, const double m5,
	const double m6, const double m7, const double m8)
{
	matrix[0] = m0; matrix[1] = m1; matrix[2] = m2;
	matrix[3] = m3; matrix[4] = m4; matrix[5] = m5;
	matrix[6] = m6; matrix[7] = m7; matrix[8] = m8;
}

Matrix3::Matrix3() 
{
	matrix[0] = 1.0; matrix[1] = 0.0; matrix[2] = 0.0;
	matrix[3] = 0.0; matrix[4] = 1.0; matrix[5] = 0.0;
	matrix[6] = 0.0; matrix[7] = 0.0; matrix[8] = 1.0;
}

void Matrix3::Identity() {
	matrix[0] = 1.0; matrix[1] = 0.0; matrix[2] = 0.0;
	matrix[3] = 0.0; matrix[4] = 1.0; matrix[5] = 0.0;
	matrix[6] = 0.0; matrix[7] = 0.0; matrix[8] = 1.0;
}

std::array<double, 9> Matrix3::GetMatrix() const
{
	return matrix;
}

Matrix3 Matrix3::operator*(const Matrix3& m) const
{
	return Matrix3(
		matrix[0] * m.matrix[0] + matrix[1] * m.matrix[3] + matrix[2] * m.matrix[6],
		matrix[0] * m.matrix[1] + matrix[1] * m.matrix[4] + matrix[2] * m.matrix[7],
		matrix[0] * m.matrix[2] + matrix[1] * m.matrix[5] + matrix[2] * m.matrix[8],
		matrix[3] * m.matrix[0] + matrix[4] * m.matrix[3] + matrix[5] * m.matrix[6],
		matrix[3] * m.matrix[1] + matrix[4] * m.matrix[4] + matrix[5] * m.matrix[7],
		matrix[3] * m.matrix[2] + matrix[4] * m.matrix[5] + matrix[5] * m.matrix[8],
		matrix[6] * m.matrix[0] + matrix[7] * m.matrix[3] + matrix[8] * m.matrix[6],
		matrix[6] * m.matrix[1] + matrix[7] * m.matrix[4] + matrix[8] * m.matrix[7],
		matrix[6] * m.matrix[2] + matrix[7] * m.matrix[5] + matrix[8] * m.matrix[8]);
}

Vector Matrix3::operator*(const Vector& v) const
{
	std::array<double, 9> mat = this->GetMatrix();
	return Vector(
		mat[0] * v[0] + mat[1] * v[1] + mat[2] * v[2],
		mat[3] * v[0] + mat[4] * v[1] + mat[5] * v[2],
		mat[6] * v[0] + mat[7] * v[1] + mat[8] * v[2]);
}

void Matrix3::Transpose()
{
	Matrix3 t(
		matrix[0], matrix[3], matrix[6],
		matrix[1], matrix[4], matrix[7],
		matrix[2], matrix[5], matrix[8]);
	this->matrix = t.matrix;
}
	
void Matrix3::Inverse() 
{
	double det = matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
		matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
		matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);

	if (det == 0.0)
		return;

	double invdet = 1.0 / det;

	Matrix3 i(
		(matrix[4] * matrix[8] - matrix[5] * matrix[7]) * invdet,
		(matrix[2] * matrix[7] - matrix[1] * matrix[8]) * invdet,
		(matrix[1] * matrix[5] - matrix[2] * matrix[4]) * invdet,
		(matrix[5] * matrix[6] - matrix[3] * matrix[8]) * invdet,
		(matrix[0] * matrix[8] - matrix[2] * matrix[6]) * invdet,
		(matrix[2] * matrix[3] - matrix[0] * matrix[5]) * invdet,
		(matrix[3] * matrix[7] - matrix[4] * matrix[6]) * invdet,
		(matrix[1] * matrix[6] - matrix[0] * matrix[7]) * invdet,
		(matrix[0] * matrix[4] - matrix[1] * matrix[3]) * invdet);
	
	this->matrix = i.matrix;
}

void Matrix3::Scale(const double s)
{
	matrix[0] *= s; matrix[1] *= s; matrix[2] *= s;
	matrix[3] *= s; matrix[4] *= s; matrix[5] *= s;
	matrix[6] *= s; matrix[7] *= s; matrix[8] *= s;
}

void Matrix3::RotateX(double theta)
{
	theta = Math::DegreeToRadian(theta);
	double c = cos(theta);
	double s = sin(theta);

	Matrix3 r(
		1.0, 0.0, 0.0,
		0.0, c, -s,
		0.0, s, c);
	this->matrix = r.matrix;
}

void Matrix3::RotateY(double theta)
{
	theta = Math::DegreeToRadian(theta);
	double c = cos(theta);
	double s = sin(theta);

	Matrix3 r(
		c, 0.0, s,
		0.0, 1.0, 0.0,
		-s, 0.0, c);

	this->matrix = r.matrix;
}

void Matrix3::RotateZ(double theta)
{
	theta = Math::DegreeToRadian(theta);
	double c = cos(theta);
	double s = sin(theta);

	Matrix3 r(
		c, -s, 0,
		s, c, 0.0,
		0.0, 0.0, 1.0);
	
	this->matrix = r.matrix;
}

void Matrix3::Rotate(const Vector u, const double theta)
 {
	double c = cos(theta);
	double s = sin(theta);
	double t = 1.0 - c;

	Matrix3 r(
		t * u[0] * u[0] + c, t * u[0] * u[1] - s * u[2], t * u[0] * u[2] + s * u[1],
		t * u[0] * u[1] + s * u[2], t * u[1] * u[1] + c, t * u[1] * u[2] - s * u[0],
		t * u[0] * u[2] - s * u[1], t * u[1] * u[2] + s * u[0], t * u[2] * u[2] + c);
	
	this->matrix = r.matrix;
}