// Pilule

#pragma once

#include <vector>
#include <iostream>

#include "mathematics.h"

class Pilule
{
protected:
    Vector a, b; //!< Lower and upper vertex.
    double r; //!< Pilule radius
public:
    //! Empty.
    Pilule() {}
    explicit Pilule(const Vector&, const double h, const double r);

    //! Empty.
    ~Pilule() {}

    // Access vertexes
    Vector& operator[] (int);
    Vector operator[] (int) const;

    // Comparison
    friend int operator==(const Pilule&, const Pilule&);
    friend int operator!=(const Pilule&, const Pilule&);

    Vector Vertex(int) const;

    double Radius() const;

    // Translation, scale
    void Translate(const Vector&);
    void Scale(double);

    friend std::ostream& operator<<(std::ostream&, const Pilule&);

public:
    static const double epsilon; //!< Internal \htmlonly\epsilon;\endhtmlonly for ray intersection tests.
};

//! Returns either end vertex of the pilule.
inline Vector& Pilule::operator[] (int i)
{
    if (i == 0) return a;
    else return b;
}

//! Overloaded.
inline Vector Pilule::operator[] (int i) const
{
    if (i == 0) return a;
    else return b;
}

/*!
\brief Returns the radius of the pilule.
*/
inline double Pilule::Radius() const
{
    return r;
}

/*!
\brief Returns the k-th vertex of the pilule.

The returned vector is computed by analysing the first three bits of k as follows:
\code
Vector vertex=Vector((k&1)?b[0]:a[0],(k&2)?b[1]:a[1],(k&4)?b[2]:a[2]);
\endcode
*/
inline Vector Pilule::Vertex(int k) const
{
    return Vector((k & 1) ? b[0] : a[0], (k & 2) ? b[1] : a[1], (k & 4) ? b[2] : a[2]);
}

/*!
\brief Check if two pilule are (strictly) equal.
\param a, b Pilules.
*/
inline int operator==(const Pilule& a, const Pilule& b)
{
    return (a.a == b.a) && (a.b == b.b);
}

/*!
\brief Check if two pilule are (strictly) different.
\param a, b Pilules.
*/
inline int operator!=(const Pilule& a, const Pilule& b)
{
    return !(a == b);
}
