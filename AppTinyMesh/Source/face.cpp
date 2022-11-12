// Self include
#include "face.h"

Face::Face(const Vector& _localUp)
{
	localUp = _localUp;
	axisA = Vector(localUp[1], localUp[2], localUp[0]);
	axisB = localUp / axisA;
}

Vector Face::getLocalUp() const
{
	return localUp;
}

Vector Face::getAxisA() const
{
	return axisA;
}

Vector Face::getAxisB() const
{
	return axisB;
}

/*!
\brief Create a terrain given an image, 2 vertices and a scale (double).

To create a terrain of an image, 2 vectices and a scale (for Z axis),
the general case, one should use:
\code
Terrain(img, Vector(0., 0., 0.), Vector(50., 50., 0.), 50);
\endcode
\param img QImage in grey level.
\param a_ Lower vertex.
\param b_ Upper vertex.
\param e Z axis scale.
*/
Face::Face(const QImage& img, float e, const Vector& _localUp) : nx(img.width()), ny(img.height()) {
    localUp = _localUp;
    axisA = Vector(localUp[1], localUp[2], localUp[0]);
    axisB = localUp / axisA;
    a = 50 * axisA;
    b = 50 * (axisB + localUp);
    
    elevation.resize(nx * ny);
    for (int x = 0; x < nx - 1; x++) {
        for (int y = 0; y < ny - 1; y++) {
            elevation[Id(x, ny - y)] = (qGreen(img.pixel(x, y)) + qBlue(img.pixel(x, y)) + qRed(img.pixel(x, y))) / (3 * e);
        }
    }


}


/*!
\brief Return index of a point given a 2D coordinate.

Return index in the one dimensional array of elevation,
 from a 2D point (x, y)

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
int Face::Id(int x, int y) const {
    return x * nx + y;
}


/*!
\brief Compute 3D Vector from a 2D coordinate paramater.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
Vector Face::Point(int x, int y) const {

    double dx = (double)x / (double)nx;
    double dy = (double)y / (double)ny;

    double px = dx * a[0] + (1 - dx) * b[0];
    double py = dy * a[1] + (1 - dy) * b[1];

    return Vector(px, py, h(x, y));
}

/*!
\brief return number of subdivision on X axis;
*/
int Face::getNx() const {
    return nx;
}

/*!
\brief return number of subdivision on Y axis;
*/
int Face::getNy() const {
    return ny;
}

/*!
\brief Return height value given a 2D coordinate.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
float Face::h(int x, int y) const {
    return elevation[Id(x, y)];
}

/*!
\brief Return the normal Vector given a 2D coordinate
 using Gradiant.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
*/
Vector Face::Gradiant(int x, int y) const {
    double gx = h(x + 1, y) - h(x - 1, y);
    double gy = h(x, y + 1) - h(x, y - 1);
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
double Face::Pente(int x, int y) const {
    Vector G = Gradiant(x, y);

    return std::max(G[0], G[1]);
}

/*!
\brief Set height to hmax of each point in the circle
 of center(x, y) and radius r if height higher than hmax.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
\param r Radius of effect.
\param hmax Maximum height.
*/
void Face::terrassement(int x, int y, int r, float hmax) {
    int r2 = r * r;

    // Ne parcours pas l'integralité du tableau si on sait
    // d'avance que le point sera trop éloigné
    for (int i = std::max(0, x - r); i < std::min(nx, x + r); i++) {
        for (int j = std::max(0, y - r); j < std::min(ny, y + r); j++) {
            if (squareDist(x, y, i, j) < r2) {
                if (elevation[Id(i, j)] > hmax) {
                    elevation[Id(i, j)] = hmax;
                }
            }
        }
    }

}

/*!
\brief Set height to hmax of each point in the circle
 of center(x, y) and radius r.

\param x Coordinate on X axis.
\param y Coordinate on Y axis.
\param r Radius of effect.
\param hmax Maximum height.
*/
void Face::terrassement2(int x, int y, int r, float hmax) {
    int r2 = r * r;

    // Ne parcours pas l'integralité du tableau si on sait
    // d'avance que le point sera trop éloigné
    for (int i = std::max(0, x - r); i < std::min(nx, x + r); i++) {
        for (int j = std::max(0, y - r); j < std::min(ny, y + r); j++) {
            if (squareDist(x, y, i, j) < r2) {
                elevation[Id(i, j)] = hmax;
            }
        }
    }

}

/*!
\brief Compute square distance between two 2D coordinates.

\param x1 Coordinate 1 on X axis.
\param y1 Coordinate 1 on Y axis.
\param x2 Coordinate 2 on X axis.
\param y2 Coordinate 2 on Y axis.
*/
int Face::squareDist(int x1, int y1, int x2, int y2) const {
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}