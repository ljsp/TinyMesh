#pragma once

#include "mathematics.h"
#include <vector>
#include <QImage>

class Terrain{
public:
    Terrain(const QImage & img, const Vector & a_, const Vector & b_, float e);

    int Id(int x, int y) const;

    Vector Point(int x, int y) const;

    Vector Gradiant(int x, int y) const;

    int getNx() const;

    int getNy() const;

    float h(int x, int y) const;

private:
    Vector a, b;
    int nx, ny;
    std::vector<float> elevation;
};
