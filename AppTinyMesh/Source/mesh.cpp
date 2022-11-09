#include "mesh.h"

/*!
\class Mesh mesh.h

\brief Core triangle mesh class.
*/



/*!
\brief Initialize the mesh to empty.
*/
Mesh::Mesh()
{
}

/*!
\brief Initialize the mesh from a list of vertices and a list of triangles.

Indices must have a size multiple of three (three for triangle vertices and three for triangle normals).

\param vertices List of geometry vertices.
\param indices List of indices wich represent the geometry triangles.
*/
Mesh::Mesh(const std::vector<Vector>& vertices, const std::vector<int>& indices) :vertices(vertices), varray(indices)
{
  normals.resize(vertices.size(), Vector::Z);
}

/*!
\brief Create the mesh.

\param vertices Array of vertices.
\param normals Array of normals.
\param va, na Array of vertex and normal indexes.
*/
Mesh::Mesh(const std::vector<Vector>& vertices, const std::vector<Vector>& normals, const std::vector<int>& va, const std::vector<int>& na) :vertices(vertices), normals(normals), varray(va), narray(na)
{
}

/*!
\brief Reserve memory for arrays.
\param nv,nn,nvi,nvn Number of vertices, normals, vertex indexes and vertex normals.
*/
void Mesh::Reserve(int nv, int nn, int nvi, int nvn)
{
  vertices.reserve(nv);
  normals.reserve(nn);
  varray.reserve(nvi);
  narray.reserve(nvn);
}

/*!
\brief Empty
*/
Mesh::~Mesh()
{
}

/*!
\brief Smooth the normals of the mesh.

This function weights the normals of the faces by their corresponding area.
\sa Triangle::AreaNormal()
*/
void Mesh::SmoothNormals()
{
  // Initialize 
  normals.resize(vertices.size(), Vector::Null);

  narray = varray;

  // Accumulate normals
  for (int i = 0; i < varray.size(); i += 3)
  {
    Vector tn = Triangle(vertices[varray.at(i)], vertices[varray.at(i + 1)], vertices[varray.at(i + 2)]).AreaNormal();
    normals[narray[i + 0]] += tn;
    normals[narray[i + 1]] += tn;
    normals[narray[i + 2]] += tn;
  }

  // Normalize 
  for (int i = 0; i < normals.size(); i++)
  {
    Normalize(normals[i]);
  }
}

/*!
\brief Add a smooth triangle to the geometry.
\param a, b, c Index of the vertices.
\param na, nb, nc Index of the normals.
*/
void Mesh::AddSmoothTriangle(int a, int na, int b, int nb, int c, int nc)
{
  varray.push_back(a);
  narray.push_back(na);
  varray.push_back(b);
  narray.push_back(nb);
  varray.push_back(c);
  narray.push_back(nc);
}

/*!
\brief Add a triangle to the geometry.
\param a, b, c Index of the vertices.
\param n Index of the normal.
*/
void Mesh::AddTriangle(int a, int b, int c, int n)
{
  varray.push_back(a);
  narray.push_back(n);
  varray.push_back(b);
  narray.push_back(n);
  varray.push_back(c);
  narray.push_back(n);
}

/*!
\brief Add a smmoth quadrangle to the geometry.

Creates two smooth triangles abc and acd.

\param a, b, c, d  Index of the vertices.
\param na, nb, nc, nd Index of the normal for all vertices.
*/
void Mesh::AddSmoothQuadrangle(int a, int na, int b, int nb, int c, int nc, int d, int nd)
{
  // First triangle
  AddSmoothTriangle(a, na, b, nb, c, nc);

  // Second triangle
  AddSmoothTriangle(a, na, c, nc, d, nd);
}

/*!
\brief Add a quadrangle to the geometry.

\param a, b, c, d  Index of the vertices and normals.
*/
void Mesh::AddQuadrangle(int a, int b, int c, int d)
{
  AddSmoothQuadrangle(a, a, b, b, c, c, d, d);
}

/*!
\brief Compute the bounding box of the object.
*/
Box Mesh::GetBox() const
{
  if (vertices.size() == 0)
  {
    return Box::Null;
  }
  return Box(vertices);
}

 // Transformations

void Mesh::Translate(const Vector& t)
{
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i] += t;
	}
}

/*!
* \brief Rotate the mesh given the x,y and z rotation angles.
* \param x, y, z Rotation angles in degrees.
*/
void Mesh::Rotate(const double angleX, const double angleY, const double angleZ)
{
    Matrix3 r,x,y,z;
	x.RotateX(angleX);
    y.RotateY(angleY);
	z.RotateZ(angleZ);
	r = x * y * z;
	
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i] = r * vertices[i];
	}
	for (int i = 0; i < normals.size(); i++)
	{
		normals[i] = r * normals[i];
	}
}

/*!
* \brief Rotate the mesh around a given vector and the angle.
* \param v Vector to rotate around.
* \param angle Angle to rotate.
*/
void Mesh::Rotate(const Vector& v, const double angle)
{
	Matrix3 r;
	r.Rotate(v, angle);

	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i] = r * vertices[i];
	}
	for (int i = 0; i < normals.size(); i++)
	{
		normals[i] = r * normals[i];
	}
}

/*!
* \brief Scales the mesh given a scaling matrix.
* \param m the scaling matrix.
*/
void Mesh::Rotate(const Matrix3& r)
{
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i] = r * vertices[i];
	}
	for (int i = 0; i < normals.size(); i++)
	{
		normals[i] = r * normals[i];
	}
}

/*!
* \brief Scale the mesh.
* \param s Scale factor.
*/
void Mesh::Scale(const Matrix3& s) 
{
	for (int i = 0; i < vertices.size(); i++)
	{
		vertices[i] = s * vertices[i];
	}
	for (int i = 0; i < normals.size(); i++)
	{
		normals[i] = s * normals[i];
	}
}

void Mesh::SphereWarp(const Sphere& s)
{
    const Vector center = s.Center();
	const double radius = s.Radius();
	
	for (int i = 0; i < vertices.size(); i++)
	{
		Vector v = center - vertices[i];
        double l = Length(v);
		if (l < radius )
		{
			double distNormalized = l / radius;
			vertices[i] += v * (1 - distNormalized);
		}
	}
}

/*!
\brief Creates an axis aligned box.

The object has 8 vertices, 6 normals and 12 triangles.
\param box The box.
*/
Mesh::Mesh(const Box& box)
{
  // Vertices
  vertices.resize(8);

  for (int i = 0; i < 8; i++)
  {
    vertices[i] = box.Vertex(i);
  }

  // Normals
  normals.push_back(Vector(-1, 0, 0));
  normals.push_back(Vector(1, 0, 0));
  normals.push_back(Vector(0, -1, 0));
  normals.push_back(Vector(0, 1, 0));
  normals.push_back(Vector(0, 0, -1));
  normals.push_back(Vector(0, 0, 1));

  // Reserve space for the triangle array
  varray.reserve(12 * 3);
  narray.reserve(12 * 3);

  AddTriangle(0, 2, 1, 4);
  AddTriangle(1, 2, 3, 4);

  AddTriangle(4, 5, 6, 5);
  AddTriangle(5, 7, 6, 5);

  AddTriangle(0, 4, 2, 0);
  AddTriangle(4, 6, 2, 0);

  AddTriangle(1, 3, 5, 1);
  AddTriangle(3, 7, 5, 1);

  AddTriangle(0, 1, 5, 2);
  AddTriangle(0, 5, 4, 2);

  AddTriangle(3, 2, 7, 3);
  AddTriangle(6, 7, 2, 3);
}

/*!
\brief Creates a face that is one face of the projection of a cube on a sphere
\param t the terrain
\param res the resolution of the shape.
*/
Mesh::Mesh(const Face& f, const int res)
{
	Vector localUp = f.getLocalUp();
    Vector axisA = f.getAxisA();
	Vector axisB = f.getAxisB();
	
	// Vertices 
	vertices.resize(res * res);
    normals.resize(res * res);
	varray.resize((res - 1) * (res - 1) * 6);
	narray.resize((res - 1) * (res - 1) * 6);
	
    int triIndex = 0;
	
    for (int y = 0; y < res; y++)
    {
        for (int x = 0; x < res; x++)
        {
			const int i = x + y * res;
			const double u = (double)x / (double)(res - 1);
			const double v = (double)y / (double)(res - 1);
			const Vector pointOnUnitCube = localUp + axisA * (u - 0.5f) * 2 + axisB * (v - 0.5f) * 2;
            const double x2 = pointOnUnitCube[0] * pointOnUnitCube[0];
            const double y2 = pointOnUnitCube[1] * pointOnUnitCube[1];
            const double z2 = pointOnUnitCube[2] * pointOnUnitCube[2];
            const double px = pointOnUnitCube[0] * sqrt(1 - (y2 + z2) / 2 + (y2 * z2) / 3);
            const double py = pointOnUnitCube[1] * sqrt(1 - (z2 + x2) / 2 + (z2 * x2) / 3);
            const double pz = pointOnUnitCube[2] * sqrt(1 - (x2 + y2) / 2 + (x2 * y2) / 3);
            const Vector pointOnUnitSphere(px,py,pz);
            vertices[i] = pointOnUnitSphere;
			
            normals[i] = pointOnUnitSphere;
			

            if (x < res - 1 && y < res - 1) 
            {
				varray[triIndex] = i;
				varray[triIndex + 1] = i + res + 1;
				varray[triIndex + 2] = i + res;

				varray[triIndex + 3] = i;
				varray[triIndex + 4] = i + 1;
				varray[triIndex + 5] = i + res + 1;

				narray[triIndex] = i;
				narray[triIndex + 1] = i + res + 1;
				narray[triIndex + 2] = i + res;

				narray[triIndex + 3] = i;
				narray[triIndex + 4] = i + 1;
				narray[triIndex + 5] = i + res + 1;

				triIndex += 6;
            }
        }
    }
}

/*!
\brief Creates a sphere that is the projection of a cube on a sphere
\param p the planet
\param res the resolution of the shape.
*/
Mesh::Mesh(const Planet& p, const int res)
{
    Vector c = p.Center();
    double radius = p.Radius();

    std::array<Vector, 6> directions;
    directions[0] = Vector(1, 0, 0);
    directions[1] = Vector(-1, 0, 0);
    directions[2] = Vector(0, 1, 0);
    directions[3] = Vector(0, -1, 0);
    directions[4] = Vector(0, 0, 1);
    directions[5] = Vector(0, 0, -1);

    // Vertices 
    vertices.resize(res * res * 6);
    normals.resize(res * res * 6);
    varray.resize((res - 1) * (res - 1) * 6 * 6);
    narray.resize((res - 1) * (res - 1) * 6 * 6);
	
    int triIndex = 0;
	
    for (int side = 0; side < 6; side++)
    {
		Vector localUp = directions[side];
        Vector axisA = Vector(localUp[1], localUp[2], localUp[0]);
        Vector axisB = localUp / axisA;
        int offset = res * res * side;
		
        for (int y = 0; y < res; y++)
        {
            for (int x = 0; x < res; x++)
            {
				const int i = x + y * res + offset;
                const double u = (double)x / (double)(res - 1);
                const double v = (double)y / (double)(res - 1);
                const Vector pointOnUnitCube = localUp + axisA * (u - 0.5f) * 2 + axisB * (v - 0.5f) * 2;
                const double x2 = pointOnUnitCube[0] * pointOnUnitCube[0];
                const double y2 = pointOnUnitCube[1] * pointOnUnitCube[1];
                const double z2 = pointOnUnitCube[2] * pointOnUnitCube[2];
                const double px = pointOnUnitCube[0] * sqrt(1 - (y2 + z2) / 2 + (y2 * z2) / 3);
                const double py = pointOnUnitCube[1] * sqrt(1 - (z2 + x2) / 2 + (z2 * x2) / 3);
                const double pz = pointOnUnitCube[2] * sqrt(1 - (x2 + y2) / 2 + (x2 * y2) / 3);
                const Vector pointOnUnitSphere(px, py, pz);
				
                vertices[i] = pointOnUnitSphere * radius;
                normals[i] = pointOnUnitSphere * radius;

                if (x < res - 1 && y < res - 1)
                {
                    varray[triIndex] = i;
                    varray[triIndex + 1] = i + res + 1;
                    varray[triIndex + 2] = i + res;

                    varray[triIndex + 3] = i;
                    varray[triIndex + 4] = i + 1;
                    varray[triIndex + 5] = i + res + 1;

                    narray[triIndex] = i;
                    narray[triIndex + 1] = i + res + 1;
                    narray[triIndex + 2] = i + res;

                    narray[triIndex + 3] = i;
                    narray[triIndex + 4] = i + 1;
                    narray[triIndex + 5] = i + res + 1;

                    triIndex += 6;
                }
            }
        }
    }
}

/*!
\brief Creates an axis aligned disc.
\param disc the disc.
\param nbDivision the number of divisions of the shape.
*/
Mesh::Mesh(const Disc& disc, const int nbDivision)
{
    const Vector a = disc.Vertex();
    const double radius = disc.Radius();

    // Orthonormal basis
    const Vector z = Normalized(a);
    Vector x, y;
    z.Orthonormal(x, y);

    // Vertices
    const int vertexCount = nbDivision + 1; //Each division 1 vertex + 1 for the center
    vertices.reserve(vertexCount);

    // Circle slice size
    const double theta = (2 * 3.141592) / nbDivision;

    // Create Disc circle vertex
    for (int i = 0; i < nbDivision; i++)
    {
        Vector va(x * cos(theta * i) + y * sin(theta * i) + a);
        va *= radius;
        vertices.push_back(va);
    }

    // Create circle triangles
    vertices.push_back(a);
    normals.push_back(-z);
    for (int i = 0; i < nbDivision; i++)
        AddTriangle(vertices.size() - 1, i, (i + 1) % nbDivision, normals.size() - 1);
}

/*!
\brief Creates an axis aligned cone.
\param cone the cone.
\param nbDivision the number of divisions of the shape.
*/
Mesh::Mesh(const Cone& cone, const int nbDivision)
{
    const Vector a = cone.Vertex(0);
    const Vector b = cone.Vertex(1);
    const double radius = cone.Radius();

    // Orthonormal basis
    const Vector z = Normalized(b - a);
    Vector x, y;
    z.Orthonormal(x, y);

    // Vertices
    const int vertexCount = nbDivision + 2; //Each division 1 vertex + 2 for the cone tip and center
    vertices.reserve(vertexCount);

    // Circle slice size
    const double theta = (2 * 3.141592) / nbDivision;

    // Adding Vertex

    for (int i = 0; i < nbDivision; i++)
    {
        Vector va(x * cos(theta * i) * radius + y * sin(theta * i) * radius + a);
        vertices.push_back(va);
    }

    vertices.push_back(a);
    vertices.push_back(b);

    // Adding Normals 
    for (int i = 0; i < nbDivision; i++)
    {
        Vector normal = Normalized(vertices[i] - z);
        normals.push_back(normal);
    }

    normals.push_back(-z);

    // Adding Triangles
    for (int i = 0; i < nbDivision; i++)
    {
        AddTriangle(vertices.size() - 2, i, (i + 1) % nbDivision, normals.size() - 1);
    }

    //Loop for the sides triangle
    for (int i = 0; i < nbDivision; i++)
    {
        AddSmoothTriangle(vertices.size() - 1, vertices.size() - 2, i , i, ((i + 1) % nbDivision), ((i + 1) % nbDivision));
    }

}

/*!
\brief Creates an axis aligned cylinder.
\param cyl the cylinder.
\param nbDivision the number of divisions of the shape.
*/
Mesh::Mesh(const Cylinder& cyl, const int nbDivision)
{
    const Vector a = cyl.Vertex(0);
    const Vector b = cyl.Vertex(1);
    const double radius = cyl.Radius();

    // Orthonormal basis
    const Vector z = Normalized(b - a);
    Vector x, y;
    z.Orthonormal(x, y);

	
    // Vertices
    const int vertexCount = (nbDivision * 2) + 2; //Each division neads 2 vertex + 2 for the circles centers
    vertices.reserve(vertexCount); 

    // Circle slice size
    const double theta = (2 * 3.141592) / nbDivision;

    for (int i = 0; i < nbDivision; i++)
    {
        Vector va(x * cos(theta * i) * radius + y * sin(theta * i) * radius + a);
        vertices.push_back(va);
    }

    int offset = vertices.size();
    for (int i = 0; i < nbDivision; i++)
    {
        Vector vb(x * cos(theta * i) * radius + y * sin(theta * i) * radius + b);
        vertices.push_back(vb);
    }

    vertices.push_back(a);
    vertices.push_back(b);

    //Loop for the sides
    for (int i = 0; i < nbDivision * 2; i++)
    {
        Vector normal = Normalized(vertices[i] - z);
        normals.push_back(normal);
    }
	
    normals.push_back(-z);
    normals.push_back(z);

	for (int i = 0; i < nbDivision; i++)
	{
        AddSmoothTriangle(i, i, (i + 1) % nbDivision, (i + 1) % nbDivision, i + offset, i + offset);
        AddSmoothTriangle(i + offset, i + offset, ((i + 1) % nbDivision) + offset, ((i + 1) % nbDivision) + offset, (i + 1) % nbDivision, (i + 1) % nbDivision);
	}
	

    for (int i = 0; i < nbDivision; i++)
    {
        AddTriangle(vertices.size() - 2, i, (i + 1) % nbDivision, normals.size() - 2);
    }

    for (int i = 0; i < nbDivision; i++)
    {
        AddTriangle(vertices.size() - 1, i + offset, ((i + 1) % nbDivision) + offset, normals.size() - 1);
    }
}


/*!
\brief Creates a sphere.
\param s the sphere.
\param nSubdivision the number of divisions of the shape.
*/
Mesh::Mesh(const Sphere & S, const int nSubdivision){
    double r = S.Radius();
    Vector c = S.Center();
    double PI = 3.14159265358;

    int horizontalStep = nSubdivision;
    int verticalStep = nSubdivision;
    double x, y, z;

    // Ajout des 2 poles
    vertices.emplace_back(Vector(c[0], c[1], c[2]+r));
    normals.push_back(Normalized(vertices.back()));
    vertices.emplace_back(Vector(c[0], c[1], c[2]-r));
    normals.push_back(Normalized(vertices.back()));


    // h = 1 et h < horizontalStep - 1 car on a des ajouté les poles à la main avant
    // pour eviter d'avoir v meme point
    for(int h = 1; h < horizontalStep; h++){
        for(int v = 0; v < verticalStep; v++){

            x = sin(PI * (double)h/(double)horizontalStep) * cos(2*PI * (double)v/(double)verticalStep)*r + c[0];
            y = sin(PI * (double)h/(double)horizontalStep) * sin(2*PI * (double)v/(double)verticalStep)*r + c[1];
            z = cos(PI * (double)h/(double)horizontalStep)*r + c[2];

            vertices.emplace_back(Vector(x, y, z));
            normals.push_back(Normalized(vertices.back()));
        }
    }

    // Triangles des poles
    for(int v = 0; v < verticalStep - 1; v++){
        AddSmoothTriangle(0, 0, v+2, v+2, v+3, v+3);
        AddSmoothTriangle(1, 1, Vertexes() - v - 1, Vertexes() - v - 1, Vertexes() -v - 2, Vertexes() -v - 2);
    }

    AddSmoothTriangle(2, 2, 0, 0, 2 + verticalStep - 1, 2 + verticalStep - 1);
    AddSmoothTriangle(1, 1, Vertexes() - verticalStep, Vertexes() - verticalStep, Vertexes() - 1, Vertexes() - 1);



    for(int h = 0; h < horizontalStep - 2; h++){
        for(int v = 0; v < verticalStep; v++){
            int v1 = h*verticalStep + v + 2;
            int v4 = h*verticalStep + (v+1)%verticalStep + 2;
            int v2 = (h+1)*verticalStep + v+ 2;
            int v3 = (h+1)*verticalStep + (v+1)%verticalStep + 2;

            AddSmoothTriangle(v1, v1, v2, v2, v3, v3);
            AddSmoothTriangle(v4, v4, v1, v1, v3, v3);
        }
    }
}



/*!
\brief Creates a pilule.
\param p the sphere.
\param nSubdivision the number of divisions of the shape.
*/
Mesh::Mesh(const Pilule& p, const int nSubdivision){
    double r = p.Radius();
    Vector a = p[0];
    Vector b = p[1];
    double PI = 3.14159265358;

    int horizontalStep = nSubdivision/2;
    int verticalStep = nSubdivision;
    double x, y, z;



    // Creation de la 1ère 1/2 sphere
    vertices.emplace_back(Vector(a[0], a[1], a[2]-r));
    normals.push_back(Normalized(vertices.back()));



    for(int h = 1; h < horizontalStep; h++){
        for(int v = 0; v < verticalStep; v++){

            x = a[0] + sin(PI/2. * (double)h/(double)horizontalStep) * cos(2*PI * (double)v/(double)verticalStep)*r;
            y = a[1] + sin(PI/2. * (double)h/(double)horizontalStep) * sin(2*PI * (double)v/(double)verticalStep)*r;
            z = a[2] - cos(PI/2. * (double)h/(double)horizontalStep)*r;


            vertices.emplace_back(Vector(x, y, z));
            normals.push_back(Normalized(vertices.back()));
        }
    }

    // Triangles du pole
    for(int v = 0; v < verticalStep - 1; v++){
        AddSmoothTriangle( v+1, v+1, 0, 0, v+2, v+2);
    }

    AddSmoothTriangle( 0, 0, 1, 1, verticalStep, verticalStep);



    for(int h = 0; h < horizontalStep - 2; h++){
        for(int v = 0; v < verticalStep; v++){
            int v1 = h*verticalStep + v + 1;
            int v4 = h*verticalStep + (v+1)%verticalStep + 1;
            int v2 = (h+1)*verticalStep + v+ 1;
            int v3 = (h+1)*verticalStep + (v+1)%verticalStep + 1;

            AddSmoothTriangle(v2,v2, v1, v1,v3, v3);
            AddSmoothTriangle(v1, v1, v4, v4, v3, v3);
        }
    }

    // Creation de la 2ème 1/2 sphere
    int nbVertex = Vertexes();


    vertices.emplace_back(Vector(b[0], b[1], b[2]+r));
    normals.push_back(Normalized(vertices.back()));


    for(int h = 1; h < horizontalStep; h++){
        for(int v = 0; v < verticalStep; v++){

            x = b[0] + sin(PI/2. * (double)h/(double)horizontalStep) * cos(2*PI * (double)v/(double)verticalStep)*r;
            y = b[1] + sin(PI/2. * (double)h/(double)horizontalStep) * sin(2*PI * (double)v/(double)verticalStep)*r;
            z = b[2] + cos(PI/2. * (double)h/(double)horizontalStep)*r;

            vertices.emplace_back(Vector(x, y, z));
            normals.push_back(Normalized(vertices.back()));
        }
    }

    // Triangles du poles
    for(int v = 0; v < verticalStep - 1; v++){
        AddSmoothTriangle(nbVertex, nbVertex, nbVertex+ v+1, nbVertex+ v+1, nbVertex+ v+2, nbVertex+ v+2);
    }

    AddSmoothTriangle(nbVertex+ 1, nbVertex+ 1, nbVertex, nbVertex, nbVertex+ verticalStep, nbVertex+ verticalStep);



    for(int h = 0; h < horizontalStep - 2; h++){
        for(int v = 0; v < verticalStep; v++){
            int v1 = h*verticalStep + v + 1  + nbVertex;
            int v4 = h*verticalStep + (v+1)%verticalStep + 1 + nbVertex;
            int v2 = (h+1)*verticalStep + v+ 1 + nbVertex;
            int v3 = (h+1)*verticalStep + (v+1)%verticalStep + 1 + nbVertex;

            AddSmoothTriangle(v1, v1, v2, v2, v3, v3);
            AddSmoothTriangle(v4, v4, v1, v1, v3, v3);
        }
    }


    // Ajout des triangles reliant les deux 1/2 spheres

    int vertexSphere1 = Vertexes()/2 - verticalStep;
    int vertexSphere2 = Vertexes() - verticalStep;

    for(int i=0; i<verticalStep - 1; i++){
        AddSmoothTriangle(vertexSphere1 + i, vertexSphere1 + i, vertexSphere1 + i + 1, vertexSphere1 + i + 1, vertexSphere2 + i, vertexSphere2 + i);
        AddSmoothTriangle( vertexSphere2 + i, vertexSphere2 + i, vertexSphere1 + i + 1, vertexSphere1 + i + 1,vertexSphere2 + i + 1, vertexSphere2 + i + 1);
    }

    AddSmoothTriangle(vertexSphere1 + verticalStep - 1, vertexSphere1 + verticalStep - 1, vertexSphere1, vertexSphere1, vertexSphere2, vertexSphere2);
    AddSmoothTriangle(vertexSphere2, vertexSphere2, vertexSphere2 + verticalStep - 1, vertexSphere2 + verticalStep - 1, vertexSphere1 + verticalStep - 1, vertexSphere1 + verticalStep - 1);
    //AddSmoothTriangle(vertexSphere1, vertexSphere1, vertexSphere2 + verticalStep - 1, vertexSphere2 + verticalStep - 1, vertexSphere2, vertexSphere2);


}

/*!
\brief Creates a torus.
\param t the torus.
\param res the number of divisions of the shape.
*/
Mesh::Mesh(const Torus& t, const int res)
{
	double PI = 3.14159265358;
	double r = t.Radius();
	double thickness = t.Thickness();
	double x, y, z;
	int horizontalStep = res;
	int verticalStep = res;

	for (int h = 0; h < horizontalStep; h++) {
		for (int v = 0; v < verticalStep; v++) {

			x = (r + thickness * cos(2 * PI * (double)h / (double)horizontalStep)) * cos(2 * PI * (double)v / (double)verticalStep);
			y = (r + thickness * cos(2 * PI * (double)h / (double)horizontalStep)) * sin(2 * PI * (double)v / (double)verticalStep);
			z = thickness * sin(2 * PI * (double)h / (double)horizontalStep);

			vertices.emplace_back(Vector(x, y, z));
			normals.push_back(Normalized(vertices.back()));
		}
	}

	for (int h = 0; h < horizontalStep - 1; h++) {
		for (int v = 0; v < verticalStep; v++) {
			int v1 = h * verticalStep + v;
			int v4 = h * verticalStep + (v + 1) % verticalStep;
			int v2 = (h + 1) * verticalStep + v;
			int v3 = (h + 1) * verticalStep + (v + 1) % verticalStep;

            AddSmoothQuadrangle(v1, v1, v2, v2, v3, v3, v4, v4);
		}
	}

	// Ajout de la derniere ligne de triangles
	for (int v = 0; v < verticalStep - 1; v++) {
		int v1 = (horizontalStep - 1) * verticalStep + v;
		int v2 = (horizontalStep - 1) * verticalStep + v + 1;
		int v3 = v;
		int v4 = v + 1;
		
        AddSmoothQuadrangle(v4, v4, v2, v2, v1, v1, v3, v3);
	}

	// Ajout du dernier rectangle
    AddSmoothQuadrangle(
        0, 0,
        (horizontalStep - 1) * verticalStep, (horizontalStep - 1) * verticalStep,
        (horizontalStep - 1) * verticalStep + verticalStep - 1, (horizontalStep - 1) * verticalStep + verticalStep - 1,
        verticalStep - 1, verticalStep - 1);	
}



Mesh::Mesh(const Terrain & terrain){

    vertices.resize(terrain.getNx()*terrain.getNy());
    normals.resize(terrain.getNx()*terrain.getNy());

    for(int i=1; i<terrain.getNx() - 1; i++){
        for(int j=1; j<terrain.getNy() - 1; j++){
            vertices[terrain.Id(i, j)] = terrain.Point(i, j);
            normals[terrain.Id(i, j)] = Normalized(terrain.Gradiant(i, j));
        }
    }


    for(int i=1; i<terrain.getNx() - 1; i++){
        for(int j=0; j<terrain.getNy() - 2; j++){
            AddSmoothTriangle(terrain.Id(i, j), terrain.Id(i, j),
                              terrain.Id(i, j+1), terrain.Id(i, j+1),
                              terrain.Id(i+1, j+1), terrain.Id(i+1, j+1));

            AddSmoothTriangle(terrain.Id(i+1, j), terrain.Id(i+1, j),
                              terrain.Id(i, j), terrain.Id(i, j),
                              terrain.Id(i+1, j+1), terrain.Id(i+1, j+1));

        }
    }


}




/*!
\brief Scale the mesh.
\param s Scaling factor.
*/
void Mesh::Scale(double s)
{
    // Vertexes
    for (int i = 0; i < vertices.size(); i++)
    {
        vertices[i] *= s;
    }

    if (s < 0.0)
    {
        // Normals
        for (int i = 0; i < normals.size(); i++)
        {
            normals[i] = -normals[i];
        }
    }
}



#include <QtCore/QFile>
#include <QtCore/QTextStream>
#include <QtCore/QRegularExpression>
#include <QtCore/qstring.h>

void Mesh::Merge(const Mesh& mesh)
{
    int vertexOffset = this->vertices.size();
    int normalsOffset = this->normals.size();
	
	this->vertices.insert(this->vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
	this->normals.insert(this->normals.end(), mesh.normals.begin(), mesh.normals.end());


	for (int i = 0; i < mesh.varray.size(); i++)
	{
		this->varray.push_back(mesh.varray[i] + vertexOffset);
	}
	
    for (int i = 0; i < mesh.narray.size(); i++)
    {
		this->narray.push_back(mesh.narray[i] + normalsOffset);
    }
}

/*!
\brief Import a mesh from an .obj file.
\param filename File name.
*/
void Mesh::Load(const QString& filename)
{
  vertices.clear();
  normals.clear();
  varray.clear();
  narray.clear();

  QFile data(filename);

  if (!data.open(QFile::ReadOnly))
    return;
  QTextStream in(&data);

  // Set of regular expressions : Vertex, Normal, Triangle
  QRegularExpression rexv("v\\s*([-|+|\\s]\\d*\\.\\d+)\\s*([-|+|\\s]\\d*\\.\\d+)\\s*([-|+|\\s]\\d*\\.\\d+)");
  QRegularExpression rexn("vn\\s*([-|+|\\s]\\d*\\.\\d+)\\s*([-|+|\\s]\\d*\\.\\d+)\\s*([-|+|\\s]\\d*\\.\\d+)");
  QRegularExpression rext("f\\s*(\\d*)/\\d*/(\\d*)\\s*(\\d*)/\\d*/(\\d*)\\s*(\\d*)/\\d*/(\\d*)");
  while (!in.atEnd())
  {
    QString line = in.readLine();
    QRegularExpressionMatch match = rexv.match(line);
    QRegularExpressionMatch matchN = rexn.match(line);
    QRegularExpressionMatch matchT = rext.match(line);
    if (match.hasMatch())//rexv.indexIn(line, 0) > -1)
    {
      Vector q = Vector(match.captured(1).toDouble(), match.captured(2).toDouble(), match.captured(3).toDouble()); vertices.push_back(q);
    }
    else if (matchN.hasMatch())//rexn.indexIn(line, 0) > -1)
    {
      Vector q = Vector(matchN.captured(1).toDouble(), matchN.captured(2).toDouble(), matchN.captured(3).toDouble());  normals.push_back(q);
    }
    else if (matchT.hasMatch())//rext.indexIn(line, 0) > -1)
    {
      varray.push_back(matchT.captured(1).toInt() - 1);
      varray.push_back(matchT.captured(3).toInt() - 1);
      varray.push_back(matchT.captured(5).toInt() - 1);
      narray.push_back(matchT.captured(2).toInt() - 1);
      narray.push_back(matchT.captured(4).toInt() - 1);
      narray.push_back(matchT.captured(6).toInt() - 1);
    }
  }
  data.close();
}

/*!
\brief Save the mesh in .obj format, with vertices and normals.
\param url Filename.
\param meshName %Mesh name in .obj file.
*/
void Mesh::SaveObj(const QString& url, const QString& meshName) const
{
  QFile data(url);
  if (!data.open(QFile::WriteOnly))
    return;
  QTextStream out(&data);
  out << "g " << meshName << Qt::endl;
  for (int i = 0; i < vertices.size(); i++)
    out << "v " << vertices.at(i)[0] << " " << vertices.at(i)[1] << " " << vertices.at(i)[2] << QString('\n');
  for (int i = 0; i < normals.size(); i++)
    out << "vn " << normals.at(i)[0] << " " << normals.at(i)[1] << " " << normals.at(i)[2] << QString('\n');
  for (int i = 0; i < varray.size(); i += 3)
  {
    out << "f " << varray.at(i) + 1 << "//" << narray.at(i) + 1 << " "
      << varray.at(i + 1) + 1 << "//" << narray.at(i + 1) + 1 << " "
      << varray.at(i + 2) + 1 << "//" << narray.at(i + 2) + 1 << " "
      << "\n";
  }
  out.flush();
  data.close();
}

