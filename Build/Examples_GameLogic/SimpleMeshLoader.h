#pragma once
#include <nclgl\Mesh.h>
#include <nclgl\NCLDebug.h>

namespace SimpleMeshLoader
{
	Mesh* LoadMeshFile(const std::string& filename, const Matrix4& bake_transform = Matrix4())
	{
		std::ifstream file;

		file.open(filename.c_str());
		if (!file.is_open()) {
			NCLERROR("Unable to load simple mesh file - \"%s\"", filename.c_str());
			return NULL;
		}

		Mesh* m = new Mesh();
		m->type = GL_TRIANGLES;

		//First two values in the file represent the number of vertices, and number of indices
		file >> m->numVertices;
		file >> m->numIndices;

		//The next part is all the vertex positions
		m->vertices = new Vector3[m->numVertices];
		for (uint i = 0; i < m->numVertices; ++i)
		{
			Vector3& vertex = m->vertices[i];
			file >> vertex.x;
			file >> vertex.y;
			file >> vertex.z;

			vertex = bake_transform * vertex;
		}

		//The final part is all the indices
		m->indices = new uint[m->numIndices];
		for (uint i = 0; i < m->numIndices; ++i)
		{
			file >> m->indices[i];
		}

		file.close();

		m->GenerateNormals();
		m->BufferData();
		return m;
	}
};