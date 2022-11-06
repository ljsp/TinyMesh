
// Planet

#pragma once

#include <vector>
#include <iostream>
#include <array>

#include "mathematics.h"

class Planet {
protected:
    Vector center;
	const double radius = 1.0;

public:
    //! Empty.
    Planet() {}
	explicit Planet(const Vector&, const double);

	double Radius() const;
	
    Vector Center() const;

    //! Empty.
    ~Planet() {}
};