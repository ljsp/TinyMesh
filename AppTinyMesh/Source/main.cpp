#include "qte.h"
#include "terrain.h"

int main(int argc, char *argv[])
{
    //Mesh m(Cylinder(Vector(-1.0, 0.0, 0.0), Vector(1.0, 1.0, 0.0), 1), 64);
    //Mesh m(Sphere(Vector(0., 0., 0.), 5), 20);

    //Mesh m(Box(Vector(0., 0., 0.), 4.));
    //Mesh m(Pilule(Vector(0., 0., 0.), 10, 2), 20);
    /*
    QImage img;
    //img.load("../AppTinyMesh/data/heightmap.png");
    img.load("../AppTinyMesh/data/lyon.png");

    Terrain t(img, Vector(0., 0., 0.), Vector(50., 50., 0.), 50);
    // terrassement au millieu du terrain, de rayon 50 et hmax 1.;
    t.terrassement(t.getNx()/2, t.getNy()/2, 50., 1.);

    Mesh m(t);
    m.SaveObj(QString("objet.obj"), QString("cube"));

	return 0;*/

	QApplication app(argc, argv);
	MainWindow mainWin;
	mainWin.showMaximized();
	
	return app.exec();
	
}
