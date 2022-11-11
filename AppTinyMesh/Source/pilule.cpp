#include "pilule.h"

/*!
\class Pilule pilule.h
\brief A pilule.

The class stores the opposite two vectors and a radius.
The vertices of a pilule can be obtained by the Pilule::Vertex(int i)
 member function that returns vector A if i==1 or B if i==2.
 The two opposite corners can be obtained faster as follows:

 \code
Pilule pilule(Vector(0., 0., 0.), 10, 2); // Pilule
Vector a=pilule[0]; // Lower vertex
Vector b=pilule[1]; // Upper vertex
\endcode
*/

const double Pilule::epsilon = 1.0e-5; //!< Epsilon value used to check intersections and some round off errors.



/*!
\brief Create a pilule given a vector, the height and a radius.

To create a cylinder of a vector and its height
the general case, one should use:
\code
Pilule pil(Vector(0.0,0.0,0.0),2.0, 1);
\endcode
\param a bottom vertice.
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
\param pil The pilule.
*/
std::ostream& operator<<(std::ostream& s, const Pilule& pil)
{
    s << "Pilule(" << pil.a << ',' << pil.b << ")";
    return s;
}

/*!
\brief Translates a pilule.

\param t Translation vector.
*/
void Pilule::Translate(const Vector& t)
{
    a += t;
    b += t;
}

/*!
\brief Scales a pilule.

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
