// Planet

// Self include
#include "planet.h"

Planet::Planet(const Vector& c, const double r) : center(c), radius(r) {

}

Vector Planet::Center() const {
    return center;
}

double Planet::Radius() const {
	return radius;
}
