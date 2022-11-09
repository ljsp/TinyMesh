//
// Created by Stanislas BAGNOL on 03/11/2022.
//

#include "terrain.h"


Terrain::Terrain(const QImage & img, const Vector & a_, const Vector & b_, float e): a(a_), b(b_), nx(img.width()), ny(img.height()){
    elevation.resize(nx*ny);
    for(int x=0; x<nx - 1; x++){
        for(int y=0; y<ny - 1; y++){
            elevation[Id(x, ny-y)] = (qGreen(img.pixel(x, y))+qBlue(img.pixel(x, y))+qRed(img.pixel(x, y)))/(3*e);
        }
    }


}

int Terrain::Id(int x, int y) const{
    return x*nx + y;
}

Vector Terrain::Point(int x, int y) const{

    double dx = (double)x/(double)nx;
    double dy = (double)y/(double)ny;

    double px = dx*a[0] + (1-dx)*b[0];
    double py = dy*a[1] + (1-dy)*b[1];

    return Vector(px, py, h(x, y));
}

int Terrain::getNx() const {
    return nx;
}

int Terrain::getNy() const {
    return ny;
}

float Terrain::h(int x, int y) const{
    return elevation[Id(x, y)];
}

Vector Terrain::Gradiant(int x, int y) const{
    double gx = h(x+1, y) - h(x-1, y);
    double gy = h(x, y+1) - h(x, y-1);
    double gz = 0.;

    if(gx == 0 && gy == 0){
        gz = 1.;
    }

    return Vector(gx, gy, gz);
}

double Terrain::Pente(int x, int y) const {
    Vector G = Gradiant(x, y);

    return std::max(G[0], G[1]);
}

void Terrain::terrassement(int x, int y, int r, float hmax) {
    int r2 = r * r;

    for (int i = std::max(0, x - r); i < std::min(nx, x + r); i++) {
        for (int j = std::max(0, y - r); j < std::min(ny, y + r); j++) {
            if (squareDist(x, y, i, j) < r2) {
                if (elevation[Id(i, j)] > hmax) {
                    elevation[Id(i, j)] = hmax;
                }
            }
        }
    }

}

int Terrain::squareDist(int x1, int y1, int x2, int y2) const {
    return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
}
