// Disc

// Self include
#include "disc.h"

/*!
\class Disc disc.h
\brief A disc.

The class stores the center as a vector.
The vertice of a disc can be obtained by the Disc::Vertex()
*/

const double Disc::epsilon = 1.0e-5; //!< Epsilon value used to check intersections and some round off errors.


/*!
\brief Create a disc given two vectors and a radius.

Note that this constructor does not check the coordinates of the two vectors.
Therefore, the coordinates of a should be lower than those of b.

To create a disc off of two vectors a and b in
the general case, one should use:
\code
Disc disc(Vector(0.0,0.0,0.0), 1);
\endcode
\param a disc center vertice.
\param r disc radius.
*/
Disc::Disc(const Vector& a, const double r)
{
    Disc::a = a;
    Disc::r = r;
}

/*!
\brief Overloaded.
\param s Stream.
\param disc The disc.
*/
std::ostream& operator<<(std::ostream& s, const Disc& disc)
{
    s << "Disc(" << disc.a << ")";
    return s;
}

/*!
\brief Translates a disc.

\param t Translation vector.
*/
void Disc::Translate(const Vector& t)
{
    a += t;
}

/*!
\brief Scales a disc.

Note that this function handles negative coefficients in
the scaling vector (by swapping coordinates if need be).
\param s Scaling.
*/
void Disc::Scale(double s)
{
    a *= s;
}
