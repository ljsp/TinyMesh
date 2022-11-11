// Sphere

// Self include
#include "sphere.h"


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



const double Sphere::epsilon = 1.0e-5; //!< Epsilon value used to check intersections and some round off errors.


/*!
\brief Create a sphere given a vector and a radius.

To create a sphere of a vector and its radius
the general case, one should use:
\code
Sphere s(Vector(0.0,0.0,0.0), 2.0);
\endcode
\param c center vertice.
\param r sphere radius.
*/
Sphere::Sphere(const Vector & c, double r) : center(c), radius(r){

}



/*!
\brief Overloaded.
\param s Stream.
\param sphere The Sphere.
*/
std::ostream& operator<<(std::ostream& s, const Sphere& sphere)
{
    s << "Pilule(" << sphere.center << ',' << sphere.radius << ")";
    return s;
}

/*!
\brief Translates a sphere.

\param t Translation vector.
*/
void Sphere::Translate(const Vector& t)
{
    center += t;
}

/*!
\brief Scales a sphere.

\param s Scaling.
*/
void Sphere::Scale(double s)
{
    center *= s;
}