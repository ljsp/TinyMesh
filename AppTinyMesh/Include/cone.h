#pragma once
// Cone

#pragma once

#include <vector>
#include <iostream>

#include "mathematics.h"

class Cone 
{
protected:
	Vector a, b; //!< Lower and upper vertex.
	double r; //!< Cone radius
public:
	//! Empty.
	Cone() {}
	explicit Cone(const Vector&, const Vector&, const double r);

	//! Empty.
	~Cone() {}

	// Access vertexes
	Vector& operator[] (int);
	Vector operator[] (int) const;

	// Comparison
	friend int operator==(const Cone&, const Cone&);
	friend int operator!=(const Cone&, const Cone&);

	Vector Vertex(int) const;

	double Radius() const;

	// Translation, scale
	void Translate(const Vector&);
	void Scale(double);

	friend std::ostream& operator<<(std::ostream&, const Cone&);

public:
	static const double epsilon; //!< Internal \htmlonly\epsilon;\endhtmlonly for ray intersection tests.
};

//! Returns either end vertex of the cone.
inline Vector& Cone::operator[] (int i)
{
	if (i == 0) return a;
	else return b;
}

//! Overloaded.
inline Vector Cone::operator[] (int i) const
{
	if (i == 0) return a;
	else return b;
}

/*!
\brief Returns the radius of the cone.
*/
inline double Cone::Radius() const
{
	return r;
}

/*!
\brief Returns the k-th vertex of the cone.

The returned vector is computed by analysing the first three bits of k as follows:
\code
Vector vertex=Vector((k&1)?b[0]:a[0],(k&2)?b[1]:a[1],(k&4)?b[2]:a[2]);
\endcode
*/
inline Vector Cone::Vertex(int k) const
{
	return Vector((k & 1) ? b[0] : a[0], (k & 2) ? b[1] : a[1], (k & 4) ? b[2] : a[2]);
}

/*!
\brief Check if two cones are (strictly) equal.
\param a, b Cones.
*/
inline int operator==(const Cone& a, const Cone& b)
{
	return (a.a == b.a) && (a.b == b.b);
}

/*!
\brief Check if two cones are (strictly) different.
\param a, b Cones.
*/
inline int operator!=(const Cone& a, const Cone& b)
{
	return !(a == b);
}
