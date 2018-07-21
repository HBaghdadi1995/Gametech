#include "CudaCollidingParticles.cuh"


//When using the thrust library, anytime you want to use an anonomous function
// to process the array, you need to wrap it in a struct and pass that in instead.
//For example, this method is triggered by thrust for each element in our Particle
// array, and the output will is stored automatically in our openGL particle array.
struct CopyToOpenGL
{
	__host__ __device__
	float3 operator()(const Particle& p)
	{
		//Particles are go from 0 - grid width, and we want it to be centred on 0,0,0!
		const float world_dim = PARTICLE_GRID_SIZE * PARTICLE_GRID_CELL_SIZE;
		const float3 world_offset = make_float3(world_dim * 0.5f, 0.0f, world_dim * 0.5f);
		
		float3 centred_pos = p._pos - world_offset;
		return make_float3(centred_pos.x, centred_pos.y, centred_pos.z);
	}
};



/****************************
*** ALGORITHM EXPLANATION ***
*****************************/
//Parallel collision resolution:
//  - Making any serial algorithm parallel is very hard, and what
//    will almost certainly take up 99% of any GPU project. For this
//    example, collision resolution, we just take a n*2 approach.
//    Simply: For each collision, we process it twice, once for object A
//    and once for object B. The reason we do this is to avoid reading and 
//    writing to the same data at the same time (e.g. our physics constraints in parallel).
//    Instead, we allocate a thread to each particle, let it sum up all of the 'resolution'
//    forces acting on it from nearby collisions.
//    
//    On paper, this is just a much slower version of our CPU solver, though when split
//    onto hundreds of cores is still much faster than our CPU approach.

//How do we know which particles are neighbours?
//   - To do the collision resolution above, we need to know for each particle
//     which other particles are nearby and possibly colliding. To accomplish this
//     we do use a bucket sort. We generate a large 3D grid of cells and put each particle
//     into it's corresponding cell, resulting in finding all nearby particles a quick search
//     around the current and neighbouring grid cells and all their contained particles.
//
//If we have a fixed grid (like a texture) how do we place more than one particle in a single cell?
//   - Instead of having a static grid array, each grid cell just contains a start and end index which
//     points into the particle array. To generate this, we have to do a couple of steps:-
//		1: For each particle, compute it's grid cell index
//      2: Sort the particles by their grid cell indices
//      3. Run through the grid cell indices and save the 'start' of any grid cell change into our grid array
//      4. Run through the grid cell indices and save the 'end' of any grid cell change into our grid array
//

//-Footnote-
//       The result of this final codebase is actually very similar the CUDA "particles" example that comes
//       packaged with the samples. Their implementation is a bit faster, sorting lookups over entire particles
//       and using spring forces to resolve collisions in a more stable manner. If your interested, it's definetely
//       worth a look.
//
//       Another thing, for those that are interested, is a more descriptive explanation of how this works. It isn't
//       done exactly as mentioned in the article, as we don't create 8 seperate update kernels and instead just process
//       each collision pair twice. Though it explains the process much better, and is a more elegant solution to collision
//       resolution.
//		 https://developer.nvidia.com/gpugems/GPUGems3/gpugems3_ch32.html



 
__host__ __device__
int3 GetGridCell(const float3& pos)
{
	int3 cell;
	//Get a x,y,z cell index for the particle
	// Assumes positions go from 0 - (PARTICLE_GRID_SIZE * PARTICLE_GRID_CELL_SIZE)
	cell.x = static_cast<int>(pos.x / PARTICLE_GRID_CELL_SIZE);
	cell.y = static_cast<int>(pos.y / PARTICLE_GRID_CELL_SIZE);
	cell.z = static_cast<int>(pos.z / PARTICLE_GRID_CELL_SIZE);

	return cell;
}

__host__ __device__
uint GetGridCellHash(const int3& cell)
{
	//Generate a unique 'cell index' for the given cell.
	// - To handle 'edge' cases, we do a quick bitwise
	//   modulus to make sure all particles are correctly handled.

	int x = cell.x & (PARTICLE_GRID_SIZE - 1);
	int y = cell.y & (PARTICLE_GRID_SIZE - 1);
	int z = cell.z & (PARTICLE_GRID_SIZE - 1);

	return ((z * PARTICLE_GRID_SIZE) + x) * PARTICLE_GRID_SIZE + y;
}


//Bucket Sort: 1: For each particle, compute it's grid cell index
// Note: The other parts of the bucket sort list are all handled inside thrust library functions =]
struct GetCellGridIndex
{
	GetCellGridIndex() {}

	__host__ __device__
	uint operator()(const Particle& p) const
	{
		int3 cell = GetGridCell(p._pos);
		return GetGridCellHash(cell);
	}
};


//Given a particle p, check for and collide it with all particles in the given cell index
__device__
void CollideParticleWithCell(float baumgarte_factor, uint particle_idx, Particle& particle,	Particle& out_particle,
	int3 cell,
	Particle* all_particles, uint* grid_cell_start, uint* grid_cell_end)
{
	uint cellHash = GetGridCellHash(cell);

	//Get the start and end indices in the particle array which correspond
	// to the given grid cell
	uint arr_idx = grid_cell_start[cellHash];
	uint arr_end = grid_cell_end[cellHash];

	for (; arr_idx < arr_end; arr_idx++)
	{
		//Make sure we don't collide with ourselves!
		if (arr_idx == particle_idx)
			continue;

		Particle other_particle = all_particles[arr_idx];
		
		//Do a quick sphere-sphere test
		float3 ab = other_particle._pos - particle._pos;
		float lengthSq = dot(ab, ab);

		const float diameterSq = PARTICLE_RADIUS * PARTICLE_RADIUS * 4.f;
		if (lengthSq < diameterSq)
		{
			//We have a collision!
			float len = sqrtf(lengthSq);
			float3 abn = ab / len;

			//Direct normal collision (no friction/shear)
			float abnVel = dot(other_particle._vel - particle._vel, abn);		
			float jn = -(abnVel * (1.f + COLLISION_ELASTICITY));

			//Extra energy to overcome overlap error
			float overlap = PARTICLE_RADIUS * 2.f - len;
			float b = overlap * baumgarte_factor;
			
			//Normally we just add correctional energy (b) to our velocity,
			// but with such small particles and so many collisions this quickly gets 
			// out of control! The other way to solve positional errors is to move
			// the positions of the spheres, though this has numerous other problems and 
			// is ruins our collision neighbour checks. Though in general, velocity correction
			// adds energy and positional correction removes energy (and is unstable with the 
			// way we check collisions) so for now, we'll just use a half of each. Try messing
			// around with these values though! :)
			jn += b;
			//out_particle._pos -= abn * overlap * 0.5f; //Half positional correction, half because were only applying to A and not A + B
			
			
			jn = max(jn, 0.0f);
			//We just assume each particle is the same mass, so half the velocity change is applied to each.
			out_particle._vel -= abn * (jn * 0.5f);
		}

	}
}

__global__
void CollideParticles(float baumgarte_factor, uint num_particles, Particle* particles, Particle* out_particles, uint* grid_cell_start, uint* grid_cell_end)
{
	uint index = blockIdx.x*blockDim.x + threadIdx.x;
	if (index >= num_particles)
		return;

	//For each particle, check for and collide it with all neighbouring particles.
	//  - As we know the particle radius is never larger than the grid cell size we only
	//    ever have to check in a one cell radius around (and including) our grid cell.
	Particle p = particles[index];
	Particle out_p = p;
	int3 cell = GetGridCell(p._pos);

	for (int z = -1; z <= 1; ++z)
	{
		for (int x = -1; x <= 1; ++x)
		{
			for (int y = -1; y <= 1; ++y)
			{
				int3 check_cell_idx = cell + make_int3(x, y, z);
				CollideParticleWithCell(baumgarte_factor, index, p, out_p, check_cell_idx, particles, grid_cell_start, grid_cell_end);

			}
		}
	}

	out_particles[index] = out_p;
}


// Update particle positions
// - Also handles boundary resolution. We don't want our particles
//   leaving our lookup grid.
struct UpdatePositions
{
	UpdatePositions(float dt, float3 gravity)
		: _dt(dt)
		, _gravity(gravity)
		, _gridMaxBounds(PARTICLE_GRID_SIZE * PARTICLE_GRID_CELL_SIZE - PARTICLE_RADIUS)
	{
	}

	float _dt;
	float3 _gravity;
	float _gridMaxBounds;

	__host__ __device__
		void operator()(Particle& p)
	{
		//Time integration
		p._vel += _gravity;
		p._vel *= 0.999f;

		p._pos += p._vel * _dt;



		//Out of Bounds Check
		// - Horrible branching mess... Hopefully your a better programmer than me. :(

		//X
		if (p._pos.x < PARTICLE_RADIUS)
		{
			p._pos.x = PARTICLE_RADIUS;
			p._vel.x = fabs(p._vel.x) * COLLISION_ELASTICITY;
		}
		if (p._pos.x > _gridMaxBounds)
		{
			p._pos.x = _gridMaxBounds;
			p._vel.x = -fabs(p._vel.x) * COLLISION_ELASTICITY;
		}

		//Y
		if (p._pos.y < PARTICLE_RADIUS)
		{
			p._pos.y = PARTICLE_RADIUS;
			p._vel.y = fabs(p._vel.x) * COLLISION_ELASTICITY;
		}
		if (p._pos.y > _gridMaxBounds)
		{
			p._pos.y = _gridMaxBounds;
			p._vel.y = -fabs(p._vel.x) * COLLISION_ELASTICITY;
		}

		//Z
		if (p._pos.z < PARTICLE_RADIUS)
		{
			p._pos.z = PARTICLE_RADIUS;
			p._vel.z = fabs(p._vel.x) * COLLISION_ELASTICITY;
		}
		if (p._pos.z > _gridMaxBounds)
		{
			p._pos.z = _gridMaxBounds;
			p._vel.z = -fabs(p._vel.x) * COLLISION_ELASTICITY;
		}

	}
};











//All the code below this point is ONLY executed on the CPU

CudaCollidingParticles::CudaCollidingParticles()
	: num_particles(0)
	, particles_ping(NULL)
	, cGLOutPositions(NULL)
{

}

CudaCollidingParticles::~CudaCollidingParticles()
{
	if (particles_ping)
	{
		gpuErrchk(cudaFree(particles_ping));
		gpuErrchk(cudaFree(particles_pong));
		gpuErrchk(cudaFree(particles_grid_cell_index));
		gpuErrchk(cudaFree(grid_cell_start));
		gpuErrchk(cudaFree(grid_cell_end));

		particles_ping = NULL;
	}

	if (cGLOutPositions)
	{
		gpuErrchk(cudaGraphicsUnregisterResource(cGLOutPositions));
		cGLOutPositions = NULL;
	}
}



void CudaCollidingParticles::InitializeParticleDam(int dam_width, int dam_height, int dam_depth)
{
///This function could have been a lot simpler, but I wanted nicely compacted dam... >.>
	uint num_even_rowed_particles = dam_width * dam_depth * dam_height / 2;
	num_particles = num_even_rowed_particles + (dam_width - 1) * (dam_depth - 1) * dam_height / 2;

	//Allocate Particle Arrays
	gpuErrchk(cudaMalloc(&particles_pong, num_particles * sizeof(Particle)));
	gpuErrchk(cudaMalloc(&particles_grid_cell_index, num_particles * sizeof(uint)));


	//Allocate our lookup grid
	const uint num_grid_cells = PARTICLE_GRID_SIZE*PARTICLE_GRID_SIZE*PARTICLE_GRID_SIZE;

	gpuErrchk(cudaMalloc(&grid_cell_start, num_grid_cells * sizeof(uint)));
	gpuErrchk(cudaMalloc(&grid_cell_end, num_grid_cells * sizeof(uint)));



	//Generate initial Particle data for our dam
	const float sqrt2 = sqrt(2.f);

	const float3 dam_size = make_float3(
			dam_width * PARTICLE_RADIUS * 2.f,
			dam_height * PARTICLE_RADIUS * (2.f + sqrt2) * 0.5f,
			dam_depth * PARTICLE_RADIUS * 2.f);

	const float world_dim = PARTICLE_GRID_SIZE * PARTICLE_GRID_CELL_SIZE - PARTICLE_RADIUS * 2.f;
	const float3 world_size = make_float3(world_dim, world_dim, world_dim);
	
	float3 start_offset = world_size * 0.5f - dam_size * 0.5f;
	start_offset.y = 0.0f;

	Particle* tmp_particles = new Particle[num_particles];

	//Initialize all the even rows of the dam
	for (int y = 0; y < dam_height / 2; y++)
	{
		for (int z = 0; z < dam_depth; ++z)
		{
			for (int x = 0; x < dam_width; ++x)
			{
				Particle p;
				p._vel = make_float3(0.f, 0.f, 0.f);

				p._pos = PARTICLE_RADIUS * make_float3(
					1.0f + x * 2.f,
					1.0f + y * (2.f + sqrt2),
					1.0f + z * 2.f
				);
				p._pos += start_offset;

				int idx = ((y * dam_depth) + z) * dam_width + x;
				tmp_particles[idx] = p;
			}	
		}
	}

	//Initialize all the odd rows of the dam
	for (int y = 0; y < dam_height / 2; y++)
	{
		for (int z = 0; z < dam_depth - 1; ++z)
		{
			for (int x = 0; x < dam_width - 1; ++x)
			{
				Particle p;
				p._vel = make_float3(0.f, 0.f, 0.f);

				p._pos = PARTICLE_RADIUS * make_float3(
					2.f + x * 2.f,
					(1.f + sqrt2) + y * (2.f + sqrt2),
					2.f + z * 2.f
				);
				p._pos += start_offset;

				int idx = ((y * (dam_depth-1)) + z) * (dam_width-1) + x;
				tmp_particles[num_even_rowed_particles + idx] = p;
			}
		}
	}

	gpuErrchk(cudaMalloc(&particles_ping, num_particles * sizeof(Particle)));
	gpuErrchk(cudaMemcpy(particles_ping, tmp_particles, num_particles * sizeof(Particle), cudaMemcpyHostToDevice));

	delete[] tmp_particles;
}

void CudaCollidingParticles::InitializeOpenGLVertexBuffer(GLuint buffer_idx)
{
	//As the number of particles in this example is generated by the above function, the
	// opengl array has to be allocated after and initialized here later.
	gpuErrchk(cudaGraphicsGLRegisterBuffer(&cGLOutPositions, buffer_idx, cudaGraphicsMapFlagsNone));
}

void CudaCollidingParticles::UpdateParticles(float dt)
{
	//See "ALGORITHM EXPLANATION" (top of this file) for info on what is meant to be happening here.

	//Note: Gravity here is tiny! The reason being that of stability, as the particles themselves are
	// small, and the timestep is comparitively massive, we need to make sure the maximum movement
	// of each particle per timestep is small. Try messing around with it, it's also important
	// for our CPU physics engine aswell (but hopefully never been noticed ^^ ).
	// For stability, particle systems normally use spring based collision resolution instead which
	// handles correctional energy (our baumgarte scalar) more leanently.
	const float3 gravity = make_float3(0, -0.02f, 0);
	const uint num_grid_cells = PARTICLE_GRID_SIZE*PARTICLE_GRID_SIZE*PARTICLE_GRID_SIZE;
	const float fixed_timestep = 1.0f / 60.0f;
	




	//Integrate our particles through time
	// - thrust::for_each applies a given function to each element in the array
	thrust::for_each(
		thrust::device_ptr<Particle>(particles_ping),
		thrust::device_ptr<Particle>(particles_ping + num_particles),
		UpdatePositions(fixed_timestep, gravity));

	//Generate our grid cell indices
	// - thrust::transform calls a given function on each element in the first array
	//   and outputs the result into the second array.
	thrust::transform(
		thrust::device_ptr<Particle>(particles_ping),
		thrust::device_ptr<Particle>(particles_ping + num_particles),
		thrust::device_ptr<uint>(particles_grid_cell_index),
		GetCellGridIndex());


	//Sort our Particles based on their grid cell indices
	// - thrust::sort_by_key sorts both keys and values based on the key array passed in.
	// Note: Sorting is still very slow (comparitively) on the GPU and is one case where the
	//       CPU is still often faster. However, copying all our data back to the host, sorting
	//       and copying back to the device is not a feasible option. Though it's something
	//       to keep in mind when doing your own algorithms.
	thrust::sort_by_key(
		thrust::device_ptr<uint>(particles_grid_cell_index),
		thrust::device_ptr<uint>(particles_grid_cell_index + num_particles),
		thrust::device_ptr<Particle>(particles_ping));

	//Compute grid cell start indices
	// - Runs through the list of particle grid cell indices, and saves for each
	//   grid cell the point in the array where it first appears.
	thrust::counting_iterator<uint> search_begin(0u);
	thrust::lower_bound(
		thrust::device_ptr<uint>(particles_grid_cell_index),
		thrust::device_ptr<uint>(particles_grid_cell_index + num_particles),
		search_begin,
		search_begin + num_grid_cells,
		thrust::device_ptr<uint>(grid_cell_start));

	//Compute grid cell end indices
	// - Runs through the list of particle grid cell indices, and saves for each
	//   grid cell the point in the array where it last appears (+1).
	thrust::upper_bound(
		thrust::device_ptr<uint>(particles_grid_cell_index),
		thrust::device_ptr<uint>(particles_grid_cell_index + num_particles),
		search_begin,
		search_begin + num_grid_cells,
		thrust::device_ptr<uint>(grid_cell_end));


	//Handle our collision resolution
	// - For each particle, check and handle collisions with all neighbouring particles.
	// Thrust?? - To my knowledge, thrust doesn't allow you raw array access. Everything must be
	//            done with iterators - Which can be used for this function, but for me, was
	//            easier just to write our own kernel and just access the particle array directly.
	dim3 block(64, 1, 1);
	dim3 grid((num_particles + block.x - 1) / block.x, 1, 1);
	float baumgarte_factor = 0.05f / fixed_timestep;

	for (int i = 0; i < 10; ++i)
	{
		CollideParticles<<< grid, block >>>(baumgarte_factor, num_particles, particles_ping, particles_pong, grid_cell_start, grid_cell_end);
		std::swap(particles_ping, particles_pong);
		
		//Should really do boundary check's here...
	}


	//Finally, copy our particle positions to openGL to be renderered as particles.
	size_t tmpVertexPtrSize;
	float3 *tmpVertexPtr;
	gpuErrchk(cudaGraphicsMapResources(1, &cGLOutPositions, 0));
	gpuErrchk(cudaGraphicsResourceGetMappedPointer((void **)&tmpVertexPtr, &tmpVertexPtrSize, cGLOutPositions));

	if (tmpVertexPtrSize < num_particles * sizeof(float3))
	{
		NCLERROR("OpenGL vertex buffer not large enough to encompass all our particles!");
		return;
	}

	thrust::transform(
		thrust::device_ptr<Particle>(particles_ping),
		thrust::device_ptr<Particle>(particles_ping + num_particles),
		thrust::device_ptr<float3>(tmpVertexPtr),
		CopyToOpenGL());

	gpuErrchk(cudaGraphicsUnmapResources(1, &cGLOutPositions, 0));
}