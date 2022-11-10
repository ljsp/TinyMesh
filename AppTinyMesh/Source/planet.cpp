// Planet

// Self include
#include "planet.h"

/*!
\brief create a cubeSphere (planet)
\param c the center of the planet
\param r the radius
*/
Planet::Planet(const Vector& c, const double r) : center(c), radius(r) {

}

/*!
\brief Return the center of the planet.
*/
Vector Planet::Center() const {
    return center;
}

/*!
\brief Return the radius of the planet.
*/
double Planet::Radius() const {
	return radius;
}
