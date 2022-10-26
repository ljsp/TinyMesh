// Torus

// Self include
#include "torus.h"

Torus::Torus(const Vector& c, double r, double t) : center(c), radius(r), thickness(t) {

}


double Torus::Radius() const {
    return radius;
}

double Torus::Thickness() const {
	return thickness;
}

Vector Torus::Center() const {
    return center;
}
