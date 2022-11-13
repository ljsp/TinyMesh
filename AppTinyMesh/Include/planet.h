
// Planet

#pragma once

#include <vector>
#include <iostream>
#include <array>
#include "mathematics.h"
#include <QtGui/qimage.h>

class Planet {
protected:
    Vector center;
	const double radius = 1.0;

	std::array<Vector, 6> localsUp;
	std::array<Vector, 6> axisAs;
	std::array<Vector, 6> axisBs;
	std::array<Vector,6> a; //!< Lower vertex.
	std::array<Vector, 6> b; //!< Upper vertex.
	int nx;   //!< X axis number of subdivision
	int ny;   //!< Y axis number of subdivision
	std::vector<float> elevation; //!< Z height value for each point

public:
    //! Empty.
    Planet() {}
	explicit Planet(const Vector&, const double);
	explicit Planet(const Vector&, const double, const std::array<QImage,6> img, float e);

	double Radius() const;
	
    Vector Center() const;

	std::array<Vector, 6> getLocalsUp() const;
	std::array<Vector, 6> getAxisAs() const;
	std::array<Vector, 6> getAxisBs() const;


    //! Empty.
    ~Planet() {}

	int Id(int i,int x, int y) const;

	Vector Point(int i, int x, int y) const;

	Vector Gradiant(int i, int x, int y) const;

	int getNx() const;

	int getNy() const;

	float h(int i, int x, int y) const;

	double Pente(int i, int x, int y) const;

};