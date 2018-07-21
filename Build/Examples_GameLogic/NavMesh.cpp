#include "NavMesh.h"
#include <nclgl\NCLDebug.h>
#include <nclgl\Plane.h>
#include <ncltech\GeometryUtils.h>
#include <fstream>

NavMesh::NavMesh(const std::string& filename, Matrix4 transform)
	: navTris(NULL)
	, numNavVertices(0)
	, numNavTris(0)

{
	LoadNavMeshFromMesh(filename, transform);
}

NavMesh::~NavMesh()
{
	if (navTris)
	{
		delete[] navTris;
		navTris = NULL;
		numNavTris = 0;
	}

	if (numNavVertices)
	{
		for (NavVertex* vert : navVertices)
		{
			delete vert;
		}
		navVertices.clear();
		numNavVertices = 0;
	}
}

bool NavMesh::PointInTriangle(const NavTri* tri, const Vector3& point)
{
	//Code taken from "Realtime Collision Detection"

	// Compute vectors        
	Vector3 v0 = tri->_c->_pos - tri->_a->_pos;
	Vector3 v1 = tri->_b->_pos - tri->_a->_pos;
	Vector3 v2 = point - tri->_a->_pos;

	// Compute dot products
	double dot00 = (double)Vector3::Dot(v0, v0);
	double dot01 = (double)Vector3::Dot(v0, v1);
	double dot02 = (double)Vector3::Dot(v0, v2);
	double dot11 = (double)Vector3::Dot(v1, v1);
	double dot12 = (double)Vector3::Dot(v1, v2);

	// Compute barycentric coordinates
	double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
	double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	// Check if point is in triangle
	return (u >= 0.0) && (v >= 0.0) && (u + v < 1.0);
}

NavTri* NavMesh::GetFirstTriThatIntersectsLine(const Vector3& line_start, const Vector3& line_end, Vector3& out_point_on_tri)
{
	Vector3 pointOnPlane;
	for (uint i = 0; i < numNavTris; ++i)
	{
		NavTri& tri = navTris[i];

		Plane p = Plane(tri.normal, -Vector3::Dot(tri.normal, tri._a->_pos));
		if (p.PointInPlane(line_start) != p.PointInPlane(line_end))
		{
			if (GeometryUtils::PlaneEdgeIntersection(p, line_start, line_end, pointOnPlane))
			{
				if (PointInTriangle(&tri, pointOnPlane))
				{
					out_point_on_tri = pointOnPlane;
					return &tri;
				}
			}
		}
	}

	return NULL;
}


void NavMesh::DebugDraw()
{
	const float edge_width = 0.005f;
	const Vector4 edge_color = Vector4(0.25f, 0.3f, 1.0f, 1.0f);
	const Vector4 adj_color = Vector4(1.0f, 0.8f, 0.25f, 1.0f);
	const Vector4 tri_color = Vector4(0.3f, 0.6f, 1.0f, 0.5f);
	
	for (uint i = 0; i < numNavTris; ++i)
	{
		NavTri& tri = navTris[i];

		Vector3 vA = tri._a->_pos;
		Vector3 vB = tri._b->_pos;
		Vector3 vC = tri._c->_pos;

		//Draw triangle (CCW + CW)
		NCLDebug::DrawTriangle(vA, vB, vC, tri_color);
		NCLDebug::DrawTriangle(vB, vA, vC, tri_color);

		//Offset here just to move the lines 'up' and not z-fight the triangles themselves
		Vector3 offset = Vector3::Cross(vB - vA, vC - vA).Normalise() * edge_width;

		//Draw edges (once!)
		if (!tri._adjTris[0].adjTri || (tri._adjTris[0].adjTri - navTris) > i)
			NCLDebug::DrawHairLine(vA + offset, vB + offset, edge_color);

		if (!tri._adjTris[1].adjTri || (tri._adjTris[1].adjTri - navTris) > i)
			NCLDebug::DrawHairLine(vB + offset, vC + offset, edge_color);

		if (!tri._adjTris[2].adjTri || (tri._adjTris[2].adjTri - navTris) > i)
			NCLDebug::DrawHairLine(vC + offset, vA + offset, edge_color);

//#define NAVMESH_DRAW_ADJACENCIES
#ifdef NAVMESH_DRAW_ADJACENCIES
		//Draw triangle adjacencies. Will draw a line from the triangle's
		// centre to the centre of any edge which connects it to another triangle in the mesh.
		Vector3 centre_i = (vA + vB + vC) * 0.3333334f;
		for (uint j = 0; j < 3; ++j)
		{
			if (tri._adjTris[j].adjTri)
			{
				Vector3 edge_centre = (tri._verts[j]->_pos + tri._verts[(j + 1) % 3]->_pos) * 0.5f;
				NCLDebug::DrawThickLine(centre_i + offset, edge_centre + offset, edge_width, adj_color);
			}
		}
#endif
	}
}

void NavMesh::LoadMeshFile(const std::string& filename, const Matrix4& transform, uint& numVerts, uint& numIndices, Vector3** vertices, uint** indices)
{
	std::ifstream file;

	file.open(filename.c_str());
	if (!file.is_open()) {
		NCLERROR("NavMesh: Unable to load file - \"%s\"", filename.c_str());
		return;
	}

	//First two values in the file represent the number of vertices, and number of indices
	file >> numVerts;
	file >> numIndices;

	//The next part is all the vertex positions
	(*vertices) = new Vector3[numVerts];
	for (uint i = 0; i < numVerts; ++i)
	{
		Vector3& vertex = (*vertices)[i];
		file >> vertex.x;
		file >> vertex.y;
		file >> vertex.z;

		vertex = transform * vertex;
	}

	//The final part is all the indices
	(*indices) = new uint[numIndices];
	for (uint i = 0; i < numIndices; ++i)
	{
		file >> (*indices)[i];
	}

	file.close();
}

void NavMesh::ValidateTriEdges(NavTri& triA, NavTri& triB, uint edgeA, uint edgeB)
{
	NavVertex* edgeA_start = triA._verts[edgeA];
	NavVertex* edgeA_end = triA._verts[(edgeA + 1) % 3];

	NavVertex* edgeB_start = triB._verts[edgeB];
	NavVertex* edgeB_end = triB._verts[(edgeB + 1) % 3];

	//Checks indices are the same
	if ((edgeA_start == edgeB_start && edgeA_end == edgeB_end) ||
		(edgeA_start == edgeB_end && edgeA_end == edgeB_start))
	{
		NavTriAdjacency& triAdjacencyA = triA._adjTris[edgeA];
		NavTriAdjacency& triAdjacencyB = triB._adjTris[edgeB];

		triAdjacencyA.adjTri = &triB;
		triAdjacencyA.adjTriEdge = edgeB;

		triAdjacencyB.adjTri = &triA;
		triAdjacencyB.adjTriEdge = edgeA;
	}
}

void NavMesh::BuildNavMeshFromTris(uint nVerts, uint nIndices, Vector3* vertices, uint* indices)
{
	//Populate our triangles with vertex data
	numNavTris = nIndices / 3;
	navTris = new NavTri[numNavTris];

	//Because most of the vertices are actually duplicated in the mesh, we need to do a full
	// position check rather than just comparing the indices. It also results in an unknown
	// number of unique vertices, as the actual number of NavVertices can be anything up to
	// nVerts.
	auto GetClosestOrNewVertex = [&](const Vector3& point) -> NavVertex*
	{
		const float max_allowed_dist_sq = VERTEX_SNAP_DISTANCE * VERTEX_SNAP_DISTANCE;
		for (NavVertex* vert : navVertices)
		{
			Vector3 diff = vert->_pos - point;
			if (fabs(Vector3::Dot(diff, diff)) < max_allowed_dist_sq)
			{
				return vert;
			}
		}

		NavVertex* out = new NavVertex();
		out->_pos = point;
		navVertices.push_back(out);
		return out;
	};


	for (uint i = 0; i < numNavTris; ++i)
	{
		Vector3 vA = vertices[indices[i * 3]];
		Vector3 vB = vertices[indices[i * 3 + 1]];
		Vector3 vC = vertices[indices[i * 3 + 2]];

		NavTri& tri = navTris[i];
		tri = NavTri();

		tri._a = GetClosestOrNewVertex(vA);
		tri._b = GetClosestOrNewVertex(vB);
		tri._c = GetClosestOrNewVertex(vC);

		tri._a->_tris.push_back(NavVertexTriLookup(&tri, 0));
		tri._b->_tris.push_back(NavVertexTriLookup(&tri, 1));
		tri._c->_tris.push_back(NavVertexTriLookup(&tri, 2));

		tri.normal = Vector3::Cross(vB - vA, vC - vA).Normalise();
	}



	//Compute triangle adjacency
	// - This is a horrible N^2 approach which just checks every triangle edge against every other edge.
	//   Though, if this was baked into the mesh, it wouldn't be a problem as we could just precompute this
	//   and forget about it's cost.
	for (uint i = 0; i < numNavTris - 1; ++i) //For each triangle
	{
		for (uint j = i + 1; j < numNavTris; ++j)
		{
			for (uint edge_i = 0; edge_i < 3; ++edge_i) //For each edge inside triangle
			{
				for (uint edge_j = 0; edge_j < 3; ++edge_j)
				{
					ValidateTriEdges(navTris[i], navTris[j], edge_i, edge_j);
				}
			}
		}
	}
}

void NavMesh::LoadNavMeshFromMesh(const std::string& filename, const Matrix4& transform)
{
	uint numVerts, numIndices;
	Vector3* vertices = NULL;
	uint* indices;

	LoadMeshFile(filename, transform, numVerts, numIndices, &vertices, &indices);
	if (!vertices) 
		return; //Catch file load errors


	BuildNavMeshFromTris(numVerts, numIndices, vertices, indices);

	delete[] vertices;
	delete[] indices;
}