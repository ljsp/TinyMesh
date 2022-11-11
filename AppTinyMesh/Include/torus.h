// Torus

#pragma once

#include <vector>
#include <iostream>

#include "mathematics.h"

class Torus {
protected:
    Vector center; //!< Center Vector
	double radius; //!< Radius of the inner circle
    double thickness; //!< Thickness of the outer circle

public:
    //! Empty.
    Torus() {}
    explicit Torus(const Vector&, double, double);

    double Radius() const;

	double Thickness() const;

    Vector Center() const;

    //! Empty.
    ~Torus() {}

    // Comparison
    friend int operator==(const Torus&, const Torus&);
    friend int operator!=(const Torus&, const Torus&);


    // Translation, scale
    void Translate(const Vector&);
    void Scale(double);

    friend std::ostream& operator<<(std::ostream&, const Torus&);

public:
    static const double epsilon; //!< Internal \htmlonly\epsilon;\endhtmlonly for ray intersection tests.
};

/*!
\brief Returns the radius of the inner circle.
*/
inline double Torus::Radius() const {
    return radius;
}

/*!
\brief Returns the thickness of the outer circle
*/
inline double Torus::Thickness() const {
    return thickness;
}

/*!
\brief Returns the center vertex of the Torus.
*/
inline Vector Torus::Center() const {
    return center;
}

