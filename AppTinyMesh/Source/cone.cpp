// Cone

// Self include
#include "cone.h"

/*!
\class Cone cone.h
\brief A cone.

The class stores the opposite two corners as vectors.
The vertices of a cone can be obtained by the Cone::Vertex()
*/

const double Cone::epsilon = 1.0e-5; //!< Epsilon value used to check intersections and some round off errors.


/*!
\brief Create a cone given two vectors and a radius.

Note that this constructor does not check the coordinates of the two vectors.
Therefore, the coordinates of a should be lower than those of b.

To create a cone off of two vectors a and b in
the general case, one should use:
\code
Cone cone(Vector(0.0,0.0,0.0),Vector(1.0,1.0,1.0), 1);
\endcode
\param a,b End circles vertices.
\param r cone radius.
*/
Cone::Cone(const Vector& a, const Vector& b, const double r) : a(a), b(b), r(r)
{
	
}

Cone::Cone(const Vector& a, const double& h, const double r) : a(a), r(r)
{
    this->b = a + Vector(h, 0, 0);
}

/*!
\brief Overloaded.
\param s Stream.
\param cone The cone.
*/
std::ostream& operator<<(std::ostream& s, const Cone& cone)
{
    s << "Cone(" << cone.a << ',' << cone.b << ")";
    return s;
}

/*!
\brief Translates a cone.

\param t Translation vector.
*/
void Cone::Translate(const Vector& t)
{
    a += t;
    b += t;
}

/*!
\brief Scales a cone.

Note that this function handles negative coefficients in
the scaling vector (by swapping coordinates if need be).
\param s Scaling.
*/
void Cone::Scale(double s)
{
    a *= s;
    b *= s;

    // Swap coordinates for negative coefficients 
    if (s < 0.0)
    {
        Vector t = a;
        a = b;
        b = t;
    }
}
