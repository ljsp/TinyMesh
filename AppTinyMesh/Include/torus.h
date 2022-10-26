// Torus

#pragma once

#include <vector>
#include <iostream>

#include "mathematics.h"

class Torus {
protected:
    Vector center;
	double radius, thickness;

public:
    //! Empty.
    Torus() {}
    explicit Torus(const Vector&, double, double);

    double Radius() const;

	double Thickness() const;

    Vector Center() const;

    //! Empty.
    ~Torus() {}
};
