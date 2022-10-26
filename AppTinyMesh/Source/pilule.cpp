#include "pilule.h"

/*!
\class Pilule cylinder.h
\brief A cylinder.

The class stores the opposite two corners as vectors.
The vertices of a cylinder can be obtained by the Pilule::Vertex()
*/

const double Pilule::epsilon = 1.0e-5; //!< Epsilon value used to check intersections and some round off errors.



/*!
\brief Create a cylinder given a vector, the height and a radius.

Note that this constructor does not check the coordinates of the two vectors.
Therefore, the coordinates of a should be lower than those of b.

To create a cylinder of a vector and its height
the general case, one should use:
\code
Pilule cyl(Vector(0.0,0.0,0.0),2.0, 1);
\endcode
\param a bottom circle vertice.
\param h cylinder height.
\param r cylinder radius.
*/
Pilule::Pilule(const Vector& a, const double h, const double r)
{
    Pilule::a = a;
    Pilule::b = a + Vector(0.0, 0.0, h);
    Pilule::r = r;
}

/*!
\brief Overloaded.
\param s Stream.
\param cyl The cylinder.
*/
std::ostream& operator<<(std::ostream& s, const Pilule& cyl)
{
    s << "Pilule(" << cyl.a << ',' << cyl.b << ")";
    return s;
}

/*!
\brief Translates a cylinder.

\param t Translation vector.
*/
void Pilule::Translate(const Vector& t)
{
    a += t;
    b += t;
}

/*!
\brief Scales a cylinder.

Note that this function handles negative coefficients in
the scaling vector (by swapping coordinates if need be).
\param s Scaling.
*/
void Pilule::Scale(double s)
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
