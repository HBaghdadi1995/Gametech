#pragma once
#include "helper_math.h"
#include <GL\glew.h>
#include <nclgl\NCLDebug.h>
#include <cuda_runtime.h>
#include <cuda_gl_interop.h>


//Error checking
// - Almost all cuda functions can be wrapped inside this function to 
//   automatically catch (and throw) cuda related errors.
#define gpuErrchk(code) { if (code != cudaSuccess) { NCLERROR("CUDA Error: %s", cudaGetErrorString(code)); } }

