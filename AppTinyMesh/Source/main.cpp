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

	QImage img, img2, img3, img4, img5, img6;
	img.load("../AppTinyMesh/data/heightmap4.png");
	img2.load("../AppTinyMesh/data/heightmap4.png");
	img3.load("../AppTinyMesh/data/heightmap4.png");
	img4.load("../AppTinyMesh/data/heightmap4.png");
	img5.load("../AppTinyMesh/data/heightmap4.png");
	img6.load("../AppTinyMesh/data/heightmap4.png");

	std::array<QImage, 6> heightmaps;
	heightmaps[0] = img;
	heightmaps[1] = img2;
	heightmaps[2] = img3;
	heightmaps[3] = img4;
	heightmaps[4] = img5;
	heightmaps[5] = img6;

	Planet planet(Vector(0.0, 0.0, 0.0), 1.0, heightmaps, 50);
	Mesh planetMesh(planet);

    Mesh m(planetMesh);
    m.SaveObj(QString("objet.obj"), QString("cube"));*/

	//return 0;

	QApplication app(argc, argv);
	MainWindow mainWin;
	mainWin.showMaximized();
	
	return app.exec();

}
