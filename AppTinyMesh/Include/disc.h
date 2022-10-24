#pragma once
// Disc

#pragma once

#include <vector>
#include <iostream>

#include "mathematics.h"

class Disc
{
protected:
	Vector a; //!< Disc center vertex.
	double r; //!< Disc radius
public:
	//! Empty.
	Disc() {}
	explicit Disc(const Vector&, const double r);

	//! Empty.
	~Disc() {}

	Vector Vertex() const;

	double Radius() const;

	// Translation, scale
	void Translate(const Vector&);
	void Scale(double);

	friend std::ostream& operator<<(std::ostream&, const Disc&);

public:
	static const double epsilon; //!< Internal \htmlonly\epsilon;\endhtmlonly for ray intersection tests.
};

/*!
\brief Returns the radius of the disc.
*/
inline double Disc::Radius() const
{
	return r;
}

/*!
\brief Returns the k-th vertex of the disc.

The returned vector is computed by analysing the first three bits of k as follows:
\code
Vector vertex=Vector((k&1)?b[0]:a[0],(k&2)?b[1]:a[1],(k&4)?b[2]:a[2]);
\endcode
*/
inline Vector Disc::Vertex() const
{
	return a;
}