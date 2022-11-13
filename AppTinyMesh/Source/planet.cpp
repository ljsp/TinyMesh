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
\brief create a cubeSphere (planet)
\param c the center of the planet
\param r the radius
*/
Planet::Planet(const Vector& c, const double r, const std::array<QImage,6> heightmap, float e) : center(c), radius(r)
{
    localsUp[0] = Vector(1, 0, 0);
    localsUp[1] = Vector(-1, 0, 0);
    localsUp[2] = Vector(0, 1, 0);
    localsUp[3] = Vector(0, -1, 0);
    localsUp[4] = Vector(0, 0, 1);
    localsUp[5] = Vector(0, 0, -1);

    axisAs[0] = Vector(localsUp[0][1], localsUp[0][2], localsUp[0][0]);
    axisAs[1] = Vector(localsUp[1][1], localsUp[1][2], localsUp[1][0]);
    axisAs[2] = Vector(localsUp[2][1], localsUp[2][2], localsUp[2][0]);
    axisAs[3] = Vector(localsUp[3][1], localsUp[3][2], localsUp[3][0]);
    axisAs[4] = Vector(localsUp[4][1], localsUp[4][2], localsUp[4][0]);
    axisAs[5] = Vector(localsUp[5][1], localsUp[5][2], localsUp[5][0]);

    axisBs[0] = localsUp[0] / axisAs[0];
    axisBs[1] = localsUp[1] / axisAs[1];
    axisBs[2] = localsUp[2] / axisAs[2];
    axisBs[3] = localsUp[3] / axisAs[3];
    axisBs[4] = localsUp[4] / axisAs[4];
    axisBs[5] = localsUp[5] / axisAs[5];
    
    a[0] = 50 * axisAs[0];
    a[1] = 50 * axisAs[1];
    a[2] = 50 * axisAs[2];
    a[3] = 50 * axisAs[3];
    a[4] = 50 * axisAs[4];
    a[5] = 50 * axisAs[5];

    b[0] = 50 * (axisBs[0] + localsUp[0]);
    b[1] = 50 * (axisBs[1] + localsUp[1]);
    b[2] = 50 * (axisBs[2] + localsUp[2]);
    b[3] = 50 * (axisBs[3] + localsUp[3]);
    b[4] = 50 * (axisBs[4] + localsUp[4]);
    b[5] = 50 * (axisBs[5] + localsUp[5]);

    nx = heightmap[0].width();
    ny = heightmap[0].height();
    elevation.resize(nx * ny * 6);
    for (int i = 0; i < 6; i++) {
        for (int x = 0; x < nx ; x++) {
            for (int y = 0; y < ny; y++) {
                elevation[Id(i, x, (ny - y))] = (qGreen(heightmap[i].pixel(x, y)) + qBlue(heightmap[i].pixel(x, y)) + qRed(heightmap[i].pixel(x, y))) / (3 * e);
            }
        }
    }
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

std::array<Vector, 6> Planet::getLocalsUp() const {
    return localsUp;
}

std::array<Vector, 6> Planet::getAxisAs() const {
    return axisAs;
}
std::array<Vector, 6> Planet::getAxisBs() const {
    return axisBs;
}


/*!
\brief Return index of a point given a 2D coordinate.

Return index in the one dimensional array of elevation,
 from a 2D point (x, y)

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
int Planet::Id(int i, int x, int y) const {
    int offset = (nx - 1) * ny * i;
    return x * nx + y + offset;
}


/*!
\brief Compute 3D Vector from a 2D coordinate paramater.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
Vector Planet::Point(int i, int x, int y) const {

    double dx = (double)x / (double)nx;
    double dy = (double)y / (double)ny;

    double px = dx * a[i][0] + (1 - dx) * b[i][0];
    double py = dy * a[i][1] + (1 - dy) * b[i][1];

    switch (i) {
    case(0):
        return Vector(h(i, x, y), 0.0, 0.0);
    case(1):
        return Vector(-h(i, x, y), 0.0, 0.0);
    case(2):
        return Vector(0.0, h(i, x, y), 0.0);
    case(3):
        return Vector(0.0, - h(i, x, y), 0.0);
    case(4):
        return Vector(0.0, 0.0, h(i, x, y));
    case(5):
        return Vector(0.0, 0.0, - h(i, x, y));
    default:
        return Vector(0.0, 0.0, h(i, x, y));
    }
}

/*!
\brief return number of subdivision on X axis;
*/
int Planet::getNx() const {
    return nx;
}

/*!
\brief return number of subdivision on Y axis;
*/
int Planet::getNy() const {
    return ny;
}

/*!
\brief Return height value given a 2D coordinate.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
float Planet::h(int i, int x, int y) const {
    return elevation[Id(i,x, y)];
}

/*!
\brief Return the normal Vector given a 2D coordinate
 using Gradiant.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
Vector Planet::Gradiant(int i, int x, int y) const {
    double gx = h(i, x + 1, y) - h(i, x - 1, y);
    double gy = h(i, x, y + 1) - h(i, x, y - 1);
    double gz = 0.;

    if (gx == 0 && gy == 0) {
        gz = 1.;
    }

    return Vector(gx, gy, gz);
}

/*!
\brief Return the max incline on X or Y axis.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
double Planet::Pente(int i, int x, int y) const {
    Vector G = Gradiant(i, x, y);

    return std::max(G[0], G[1]);
}
