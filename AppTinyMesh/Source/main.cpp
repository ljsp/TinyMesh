#include "qte.h"

int main(int argc, char *argv[])
{
    //Mesh m(Cylinder(Vector(-1.0, 0.0, 0.0), Vector(1.0, 1.0, 0.0), 1), 64);
    //Mesh m(Sphere(Vector(0., 0., 0.), 5), 20);

    //Mesh m(Box(Vector(0., 0., 0.), 4.));
    //Mesh m(Pilule(Vector(0., 0., 0.), 10, 2), 20);
    //m.SaveObj(QString("objet.obj"), QString("cube"));
	
	QApplication app(argc, argv);
	MainWindow mainWin;
	mainWin.showMaximized();

	return app.exec();
	//return 0;
}
