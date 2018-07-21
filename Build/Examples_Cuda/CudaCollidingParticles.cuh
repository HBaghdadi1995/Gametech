#pragma once
#include <thrust\device_ptr.h>
#include <thrust\binary_search.h>
#include <thrust\iterator\counting_iterator.h>
#include <thrust\sort.h>
#include "CudaCommon.cuh"
#include <nclgl\common.h>

//The header files here will be including in c++ code,
// so can't have any cuda kernels within it.

//Size of our particles in world space
#define PARTICLE_RADIUS 0.1f

//defines a 3D grid of cells 128x128x128
#define PARTICLE_GRID_SIZE 128

//defines a world transform from grid to world, e.g. particles can move from 0 to (PARTICLE_GRID_SIZE * PARTICLE_GRID_CELL_SIZE)
// in each axis.
//!! This MUST be equal or larger than particle_radius for the broadphase to work !!
#define PARTICLE_GRID_CELL_SIZE 0.15f

#define COLLISION_ELASTICITY 0.5f

//#pragma pack(push, 16)
struct Particle
{
	float3 _vel;
	float padding1;
	float3 _pos;
	float padding2;
};
//#pragma pack(pop)

class CudaCollidingParticles
{
public:
	CudaCollidingParticles();
	~CudaCollidingParticles();

	uint GetNumParticles() { return num_particles; }

	void InitializeParticleDam(int dam_width, int dam_height, int dam_depth);
	void InitializeOpenGLVertexBuffer(GLuint buffer_idx);

	void UpdateParticles(float dt);

protected:
	uint num_particles;

	//Thrust is the Cuda equiv of the std library for c++, which this
	// class will (hopefully) show off. Though one thing to note is that
	// you can always cast a device pointer to a thrust array/vector<> but
	// you cant get a device pointer out of a thrust::vector<>. So I personally
	// recommend always managing your own device memory for the flexibility.
	Particle* particles_ping;
	Particle* particles_pong;

	//Used to sort the particles by their broadphase grid 
	uint* particles_grid_cell_index;

	//We will do a bucket sort to sort particles into a fixed grid broadphase,
	// and these arrays will keep track of the start and end indices in the particle
	// array each grid cell contains.
	uint* grid_cell_start;
	uint* grid_cell_end;

	//Our reference to the opengl vertex buffer
	cudaGraphicsResource* cGLOutPositions;
};