#include "Mesh.h"


void Mesh::loadOff(char* name)
{
	FILE* fPtr = fopen(name, "r");
	char str[334];

	fscanf(fPtr, "%s", str);

	int nVerts, nTris, n, i = 0;
	float x, y, z;

	fscanf(fPtr, "%d %d %d\n", &nVerts, &nTris, &n);
	while (i++ < nVerts)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addVertex(x, y, z);
	}

	while (fscanf(fPtr, "%d", &i) != EOF)
	{
		fscanf(fPtr, "%f %f %f", &x, &y, &z);
		addTriangle((int)x, (int)y, (int)z);
	}

	fclose(fPtr);
}


void Mesh::createCube(float sideLen)
{
	//coordinates
	float flbc[3] = { 0, 0, 0 }, deltaX = 0, deltaY = 0, deltaZ = 0;
	for (int v = 0; v < 8; v++)
	{
		switch (v)
		{
		case 1:
			deltaX = sideLen;
			break;
		case 2:
			deltaZ = -sideLen;
			break;
		case 3:
			deltaX = 0;
			break;
		case 4:
			deltaZ = 0;
			deltaY = sideLen;
			break;
		case 5:
			deltaX = sideLen;
			break;
		case 6:
			deltaZ = -sideLen;
			break;
		default:
			deltaX = 0;;
			break;
		}
		addVertex(flbc[0] + deltaX, flbc[1] + deltaY, flbc[2] + deltaZ);
	}

	addTriangle(0, 2, 1);
	addTriangle(0, 3, 2);

	addTriangle(1, 2, 5);
	addTriangle(2, 6, 5);

	addTriangle(2, 3, 6);
	addTriangle(3, 7, 6);

	addTriangle(3, 4, 7);
	addTriangle(3, 0, 4);

	addTriangle(4, 5, 6);
	addTriangle(4, 6, 7);

	addTriangle(0, 1, 5);
	addTriangle(0, 5, 4);
}

void Mesh::addTriangle(int v1, int v2, int v3)
{
	int idx = tris.size();
	tris.push_back(new Triangle(idx, v1, v2, v3));

	//set up structure

	verts[v1]->triList.push_back(idx);
	verts[v2]->triList.push_back(idx);
	verts[v3]->triList.push_back(idx);

	if (!makeVertsNeighbor(v1, v2))
		addEdge(v1, v2);

	if (!makeVertsNeighbor(v1, v3))
		addEdge(v1, v3);

	if (!makeVertsNeighbor(v2, v3))
		addEdge(v2, v3);

}

bool Mesh::makeVertsNeighbor(int v1i, int v2i)
{
	//returns true if v1i already neighbor w/ v2i; false o/w

	for (int i = 0; i < verts[v1i]->vertList.size(); i++)
		if (verts[v1i]->vertList[i] == v2i)
			return true;


	verts[v1i]->vertList.push_back(v2i);
	verts[v2i]->vertList.push_back(v1i);
	return false;
}

void Mesh::addVertex(float x, float y, float z)
{
	int idx = verts.size();
	float* c = new float[3];
	c[0] = x;
	c[1] = y;
	c[2] = z;

	verts.push_back(new Vertex(idx, c));
}

void Mesh::addEdge(int v1, int v2)
{
	int idx = edges.size();

	edges.push_back(new Edge(idx, v1, v2));

	verts[v1]->edgeList.push_back(idx);
	verts[v2]->edgeList.push_back(idx);
}

#include <cmath> // Include the <cmath> header for mathematical constants and functions
#include <Inventor/C/basic.h>

void Mesh::windingNumberByYusufSahillioglu(Point* pnt)
{
	const double PI = M_PI; // Define PI using the mathematical constant provided by <cmath>

	double a[3], b[3], c[3], aLen, bLen, cLen, twoPI = 2.0 * PI;
	pnt->winding = 0.0;

	for (int t = 0; t < (int)tris.size(); t++)
	{
		for (int ci = 0; ci < 3; ci++)
		{
			// Check if vertex indices are within bounds
			if (tris[t]->v1i < 0 || tris[t]->v1i >= verts.size() ||
				tris[t]->v2i < 0 || tris[t]->v2i >= verts.size() ||
				tris[t]->v3i < 0 || tris[t]->v3i >= verts.size()) {
				// Print an error message and return
				std::cerr << "Error: Vertex index out of bounds for triangle " << tris[t]->idx << std::endl;
				return;
			}

			a[ci] = verts[tris[t]->v1i]->coords[ci] - pnt->coords[ci];
			b[ci] = verts[tris[t]->v2i]->coords[ci] - pnt->coords[ci];
			c[ci] = verts[tris[t]->v3i]->coords[ci] - pnt->coords[ci];
		}

		aLen = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
		bLen = sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
		cLen = sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);

		pnt->winding += atan2(
			a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0]),
			aLen * bLen * cLen + cLen * (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) + aLen * (c[0] * b[0] + c[1] * b[1] + c[2] * b[2]) + bLen * (a[0] * c[0] + a[1] * c[1] + a[2] * c[2]));
	}

	if (pnt->winding >= twoPI)
		pnt->winding = 1.0; //inside
	else
		pnt->winding = 0.0; //outside
}