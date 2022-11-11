// Torus

// Self include
#include "torus.h"



/*!
\class Sphere sphere.h
\brief A sphere.

The class stores the center vector and the radius.
The Center vertex of the sphere can be obtained by
 the Sphere::Center() member function and the radius
 with Sphere::Radius().

 \code
Pilule s(Vector(0., 0., 0.), 10, 2); // Sphere
Vector c=s.Center(); // Center vertex
double r=s.Radius(); // Radius
\endcode
*/

const double Torus::epsilon = 1.0e-5; //!< Epsilon value used to check intersections and some round off errors.


/*!
\brief Create a torsus given a vector, a radius
 and a thickness.

To create a torus of a vector, its radius
 and its thickness the general case, one should use:
\code
Torus t(Vector(0.0,0.0,0.0), 10.0, 3.0);
\endcode
\param c center vector.
\param r torus radius.
\param t torus thickness
*/
Torus::Torus(const Vector& c, double r, double t) : center(c), radius(r), thickness(t) {

}



/*!
\brief Translates a torus.

\param t Translation vector.
*/
void Torus::Translate(const Vector& t)
{
    center += t;
}

/*!
\brief Scales a torus.

\param s Scaling.
*/
void Torus::Scale(double s)
{
    center *= s;
}

/*!
\brief Overloaded.
\param s Stream.
\param t The Torus.
*/
std::ostream& operator<<(std::ostream& s, const Torus& t)
{
    s << "Pilule(" << t.center << ',' << t.radius << ',' << t.thickness << ")";
    return s;
}