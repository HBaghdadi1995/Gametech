#pragma once
#include <nclgl\common.h>
#include <nclgl\Vector3.h>
#include <nclgl\Matrix4.h>
#include <string>
#include <vector>

//Turns out the mesh were using doesn't entirely use indices,
// some of the vertices are duplicated. So we need to do a full
// position check of the Vector3's rather than an index check
#define VERTEX_SNAP_DISTANCE 0.1f

//This is almost identicle to the GraphNode/GraphEdge from maze's, except now we
// work in terms of triangles! (yay!)
struct NavTri;
struct NavVertexTriLookup
{
	NavVertexTriLookup()
		: _parent(NULL)
		, _parentVertIdx(0)
	{}
	NavVertexTriLookup(NavTri* tri, uint vertIdx)
		: _parent(tri)
		, _parentVertIdx(vertIdx)
	{}

	NavTri* _parent;     //The triangle that contains this point
	uint _parentVertIdx; //The vertex index 0-2 that this vertex represents in the parent
};
struct NavVertex
{
	NavVertex()
		: _pos(0.f, 0.f, 0.f)
		, _tris()
	{}

	Vector3 _pos;
	std::vector<NavVertexTriLookup> _tris;
};



struct NavTriAdjacency
{
	NavTriAdjacency() : adjTri(NULL), adjTriEdge(0) {}

	NavTri* adjTri;
	uint adjTriEdge; //Edge index of the adjacent triangle that shares this edge
};

struct NavTri
{
	NavTri()
		: _a(NULL)
		, _b(NULL)
		, _c(NULL)
		, _adjTriAB()
		, _adjTriBC()
		, _adjTriCA()
		, normal(0.f, 0.f, 0.f)
	{
	}
	~NavTri()
	{
	}


	//Triangle vertices
	// Vert[0]: A world space position
	// Vert[1]: B world space position
	// Vert[2]: C world space position
	union
	{
		struct
		{
			NavVertex* _a;
			NavVertex* _b;
			NavVertex* _c;
		};
		NavVertex* _verts[3];
	};


	//Adjecent triangles (or NULL if they edge has no neighbours)
	// - _adjTris[0]: Adjecent triangle over edge: A->B
	// - _adjTris[1]: Adjecent triangle over edge: B->C
	// - _adjTris[2]: Adjecent triangle over edge: C->A
	union
	{
		struct
		{
			NavTriAdjacency _adjTriAB;
			NavTriAdjacency _adjTriBC;
			NavTriAdjacency _adjTriCA;
		};
		NavTriAdjacency _adjTris[3];
	};

	//Just here to speed up checking line/tri intersections
	Vector3 normal;
};


class NavMesh
{
public:
	NavMesh(const std::string& filename, Matrix4 transform = Matrix4());
	~NavMesh();

	uint GetNumNavTris() { return numNavTris; }
	NavTri* GetNavTriArray() { return navTris; }

	NavTri* GetFirstTriThatIntersectsLine(const Vector3& line_start, const Vector3& line_end, Vector3& out_point_on_tri);

	void DebugDraw();
protected:
	bool PointInTriangle(const NavTri* tri, const Vector3& point);

	//This can be any mesh, we could link this to OBJMesh, but for simplicity for now it's 
	// just simple file type consisting of indices/vertices
	void LoadMeshFile(const std::string& filename, const Matrix4& transform, uint& nVerts, uint& nIndices, Vector3** vertices, uint** indices);

	//Checks if the two edges are the same,
	// and if they are, sets the appropriate adjacency's in the NavTri's
	void ValidateTriEdges(NavTri& triA, NavTri& triB, uint edgeA, uint edgeB);

	//Turns mesh triangles into a navmesh triangles by working out
	// which triangles share an edge.
	void BuildNavMeshFromTris(uint nVerts, uint nIndices, Vector3* vertices, uint* indices);

	//Called to load the entire thing froms scratch
	void LoadNavMeshFromMesh(const std::string& filename, const Matrix4& transform);
protected:
	uint numNavTris;
	uint numNavVertices;

	NavTri* navTris;
	std::vector<NavVertex*> navVertices; //Because of duplicates, no easy way of knowing how many unique vertices there will be
};