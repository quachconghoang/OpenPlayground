#ifndef KERNEL_CU_H
#define KERNEL_CU_H

#include "cuda.h"
#include "cuda_runtime.h"

//#include <thrust/host_vector.h>
//#include <thrust/device_vector.h>
//#include <thrust/copy.h>
//#include <thrust/device_ptr.h>
//#include <thrust/sequence.h>

cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size);

#endif

