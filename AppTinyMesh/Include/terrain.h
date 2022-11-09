#pragma once

#include "mathematics.h"
#include <vector>
//#include <QImage>
#include <QtGui/qimage.h>

class Terrain{
public:
    Terrain(const QImage & img, const Vector & a_, const Vector & b_, float e);

    int Id(int x, int y) const;

    Vector Point(int x, int y) const;

    Vector Gradiant(int x, int y) const;

    int getNx() const;

    int getNy() const;

    float h(int x, int y) const;

    double Pente(int x, int y) const;

    void terrassement(int x, int y, int r, float hmax);

    int squareDist(int x1, int y1, int x2, int y2) const;

private:
    Vector a, b;
    int nx, ny;
    std::vector<float> elevation;
};
