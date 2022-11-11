// Sphere

#pragma once

#include <vector>
#include <iostream>

#include "mathematics.h"

class Sphere {
protected:
    Vector center;
    double radius;

public:
    //! Empty.
    Sphere() {}
    explicit Sphere(const Vector&, double);

    //! Empty.
    ~Sphere() {}

    double Radius() const;
    Vector Center() const;


    // Comparison
    friend int operator==(const Sphere&, const Sphere&);
    friend int operator!=(const Sphere&, const Sphere&);


    // Translation, scale
    void Translate(const Vector&);
    void Scale(double);

    friend std::ostream& operator<<(std::ostream&, const Sphere&);

public:
    static const double epsilon; //!< Internal \htmlonly\epsilon;\endhtmlonly for ray intersection tests.


};

/*!
\brief Returns the center vertex of the sphere.
*/
inline Vector Sphere::Center() const {
    return center;
}


/*!
\brief Returns the radius of the sphere.
*/
inline double Sphere::Radius() const
{
    return radius;
}



/*!
\brief Check if two Sphere are (strictly) equal.
\param a, b Sphere.
*/
inline int operator==(const Sphere& a, const Sphere& b)
{
    return (a.center == b.center) && (a.radius == b.radius);
}

/*!
\brief Check if two Sphere are (strictly) different.
\param a, b Sphere.
*/
inline int operator!=(const Sphere& a, const Sphere& b)
{
    return !(a == b);
}



