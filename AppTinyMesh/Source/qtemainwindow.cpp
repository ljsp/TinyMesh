#include "qte.h"
#include "implicits.h"

MainWindow::MainWindow()
{
	// Chargement de l'interface
	uiw.setupUi(this);

	// Chargement du GLWidget
	meshWidget = new MeshWidget;
	QGridLayout* GLlayout = new QGridLayout;
	GLlayout->addWidget(meshWidget, 0, 0);
	GLlayout->setContentsMargins(0, 0, 0, 0);
	uiw.widget_GL->setLayout(GLlayout);

	// Creation des connect
	CreateActions();

	meshWidget->SetCamera(Camera(Vector(10, 0, 0), Vector(0.0, 0.0, 0.0)));
}

MainWindow::~MainWindow()
{
	delete meshWidget;
}

void MainWindow::CreateActions()
{
	// Buttons
	connect(uiw.boxMesh, SIGNAL(clicked()), this, SLOT(BoxMeshExample()));
	connect(uiw.sphereImplicit, SIGNAL(clicked()), this, SLOT(SphereImplicitExample()));
	connect(uiw.discMesh, SIGNAL(clicked()), this, SLOT(DiscMeshExample()));
	connect(uiw.coneMesh, SIGNAL(clicked()), this, SLOT(ConeMeshExample()));
	connect(uiw.cylinderMesh, SIGNAL(clicked()), this, SLOT(CylinderMeshExample()));
	connect(uiw.sphereMesh, SIGNAL(clicked()), this, SLOT(SphereMeshExample()));
	connect(uiw.capsuleMesh, SIGNAL(clicked()), this, SLOT(CapsuleMeshExample()));
	connect(uiw.torusMesh, SIGNAL(clicked()), this, SLOT(TorusMeshExample()));
	connect(uiw.terrainMesh, SIGNAL(clicked()), this, SLOT(TerrainMeshExample()));
	connect(uiw.resetcameraButton, SIGNAL(clicked()), this, SLOT(ResetCamera()));
	connect(uiw.wireframe, SIGNAL(clicked()), this, SLOT(UpdateMaterial()));
	connect(uiw.radioShadingButton_1, SIGNAL(clicked()), this, SLOT(UpdateMaterial()));
	connect(uiw.radioShadingButton_2, SIGNAL(clicked()), this, SLOT(UpdateMaterial()));

	// Widget edition
	connect(meshWidget, SIGNAL(_signalEditSceneLeft(const Ray&)), this, SLOT(editingSceneLeft(const Ray&)));
	connect(meshWidget, SIGNAL(_signalEditSceneRight(const Ray&)), this, SLOT(editingSceneRight(const Ray&)));
}

void MainWindow::editingSceneLeft(const Ray&)
{
}

void MainWindow::editingSceneRight(const Ray&)
{
}

void MainWindow::BoxMeshExample()
{
	Mesh boxMesh = Mesh(Box(1.0));

	std::vector<Color> cols;
	cols.resize(boxMesh.Vertexes());
	for (int i = 0; i < cols.size(); i++)
		cols[i] = Color(double(i) / 6.0, fmod(double(i) * 39.478378, 1.0), 0.0);

	meshColor = MeshColor(boxMesh, cols, boxMesh.VertexIndexes());
	UpdateGeometry();
}

void MainWindow::SphereImplicitExample()
{
  AnalyticScalarField implicit;

  Mesh implicitMesh;
  implicit.Polygonize(31, implicitMesh, Box(2.0));

  std::vector<Color> cols;
  cols.resize(implicitMesh.Vertexes());
  for (int i = 0; i < cols.size(); i++)
    cols[i] = Color(0.8, 0.8, 0.8);

  meshColor = MeshColor(implicitMesh, cols, implicitMesh.VertexIndexes());
  UpdateGeometry();

}

void MainWindow::DiscMeshExample()
{
	Mesh discMesh = Mesh(Disc(Vector(0.1, 0.0, 0.0), 1.0), 64);

	std::vector<Color> cols;
	cols.resize(discMesh.Vertexes());
	for (int i = 0; i < cols.size(); i++)
		cols[i] = Color(double(i) / 6.0, fmod(double(i) * 39.478378, 1.0), 0.0);

	meshColor = MeshColor(discMesh, cols, discMesh.VertexIndexes());
	UpdateGeometry();
}

void MainWindow::ConeMeshExample()
{
	Mesh coneMesh = Mesh(Cone(Vector(1.0, 0.0, 0.0), Vector(-1.0, 1.0, 0.0), 1.0), 64);

	std::vector<Color> cols;
	cols.resize(coneMesh.Vertexes());
	for (int i = 0; i < cols.size(); i++)
		cols[i] = Color(double(i) / 6.0, fmod(double(i) * 39.478378, 1.0), 0.0);

	meshColor = MeshColor(coneMesh, cols, coneMesh.VertexIndexes());
	UpdateGeometry();
}

void MainWindow::CylinderMeshExample()
{
	Mesh cylinderMesh = Mesh(Cylinder(Vector(-1.0, 0.0, 0.0), Vector(1.0, 1.0, 0.0), 1), 64);
	cylinderMesh.Rotate(60.0, 0.0, 100.0);
	
	std::vector<Color> cols;
	cols.resize(cylinderMesh.Vertexes());
	for (int i = 0; i < cols.size(); i++)
		cols[i] = Color(double(i) / 6.0, fmod(double(i) * 39.478378, 1.0), 0.0);

	meshColor = MeshColor(cylinderMesh, cols, cylinderMesh.VertexIndexes());
	UpdateGeometry();
}

void MainWindow::SphereMeshExample()
{
	Mesh sphereMesh = Mesh(Sphere(Vector(0.0, 0.0, 0.0), 1.0), 64);

	std::vector<Color> cols;
	cols.resize(sphereMesh.Vertexes());
	for (int i = 0; i < cols.size(); i++)
		cols[i] = Color(double(i) / 6.0, fmod(double(i) * 39.478378, 1.0), 0.0);

	meshColor = MeshColor(sphereMesh, cols, sphereMesh.VertexIndexes());
	UpdateGeometry();
}

void MainWindow::CapsuleMeshExample()
{
	Mesh capsuleMesh = Mesh(Pilule(Vector(0.0, 0.0, -1.0), 2, 1), 32);

	std::vector<Color> cols;
	cols.resize(capsuleMesh.Vertexes());
	for (int i = 0; i < cols.size(); i++)
		cols[i] = Color(double(i) / 6.0, fmod(double(i) * 39.478378, 1.0), 0.0);

	meshColor = MeshColor(capsuleMesh, cols, capsuleMesh.VertexIndexes());
	UpdateGeometry();
}

void MainWindow::TorusMeshExample()
{
	Mesh torusMesh = Mesh(Torus(Vector(0.0, 0.0, 0.0), 1.0, 0.5), 64);

	std::vector<Color> cols;
	cols.resize(torusMesh.Vertexes());
	for (int i = 0; i < cols.size(); i++)
		cols[i] = Color(double(i) / 6.0, fmod(double(i) * 39.478378, 1.0), 0.0);

	meshColor = MeshColor(torusMesh, cols, torusMesh.VertexIndexes());
	UpdateGeometry();
}

void MainWindow::TerrainMeshExample()
{
	Mesh terrainMesh = Mesh(Terrain(Vector(1.0, 1.0, 0.0)), 128);

	std::vector<Color> cols;
	cols.resize(terrainMesh.Vertexes());
	for (int i = 0; i < cols.size(); i++)
		cols[i] = Color(double(i) / 6.0, fmod(double(i) * 39.478378, 1.0), 0.0);

	meshColor = MeshColor(terrainMesh, cols, terrainMesh.VertexIndexes());
	UpdateGeometry();
}

void MainWindow::UpdateGeometry()
{
	meshWidget->ClearAll();
	meshWidget->AddMesh("BoxMesh", meshColor);

	uiw.lineEdit->setText(QString::number(meshColor.Vertexes()));
	uiw.lineEdit_2->setText(QString::number(meshColor.Triangles()));

	UpdateMaterial();
}

void MainWindow::UpdateMaterial()
{
	meshWidget->UseWireframeGlobal(uiw.wireframe->isChecked());

	if (uiw.radioShadingButton_1->isChecked())
		meshWidget->SetMaterialGlobal(MeshMaterial::Normal);
	else
		meshWidget->SetMaterialGlobal(MeshMaterial::Color);
}

void MainWindow::ResetCamera()
{
	meshWidget->SetCamera(Camera(Vector(-10.0), Vector(0.0)));
}
