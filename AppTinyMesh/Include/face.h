#pragma once

// Terrain 

#include <vector>
#include <iostream>
#include "mathematics.h"
#include <QtGui/qimage.h>

class Face
{
public:
	Face(const Vector&);
	Face(const QImage& img, float e, const Vector&);
	Vector getLocalUp() const;
	Vector getAxisA() const;
	Vector getAxisB() const;
	~Face() {}


	int Id(int x, int y) const;

	Vector Point(int x, int y) const;

	Vector Gradiant(int x, int y) const;

	int getNx() const;

	int getNy() const;

	float h(int x, int y) const;

	double Pente(int x, int y) const;

	void terrassement(int x, int y, int r, float hmax);
	void terrassement2(int x, int y, int r, float hmax);

	int squareDist(int x1, int y1, int x2, int y2) const;

private:
	Vector localUp;
	Vector axisA;
	Vector axisB;

	Vector a; //!< Lower vertex.
	Vector b; //!< Upper vertex.
	int nx;   //!< X axis number of subdivision
	int ny;   //!< Y axis number of subdivision
	std::vector<float> elevation; //!< Z height value for each point
};
	
