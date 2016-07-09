#ifndef KERNEL_CU_H
#define KERNEL_CU_H

#include "cuda.h"
#include "cuda_runtime.h"

cudaError_t addWithCuda(int *c, const int *a, const int *b, unsigned int size);

#endif

