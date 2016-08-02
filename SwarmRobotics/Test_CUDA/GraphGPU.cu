#include "GraphGPU.h"

#include "cuda.h"
#include "cuda_runtime.h"
#include "cuda_runtime_api.h"
#include "device_launch_parameters.h"

#include "thrust/device_ptr.h"
#include "thrust/device_malloc.h"
#include "thrust/device_free.h"

#include "thrust/copy.h"
#include "thrust/fill.h"
#include "thrust/sequence.h"
#include "thrust/sort.h"

void test_Thrust()
{
	thrust::device_vector<int> D(10, 0);
	thrust::sequence(D.begin(), D.end());
	for (int i = 0; i < D.size(); i++)
		std::cout << "D[" << i << "] = " << D[i] << std::endl;
}

void test_Thrust_v1()
{
	// H has storage for 4 integers
	thrust::host_vector<int> H(4);
	// initialize individual elements
	H[0] = 14;
	H[1] = 20;
	H[2] = 38;
	H[3] = 46;
	std::cout << "H has size " << H.size() << std::endl;
	for (int i = 0; i < H.size(); i++)
		std::cout << "H[" << i << "] = " << H[i] << std::endl;

	H.resize(2);
	std::cout << "H now has size " << H.size() << std::endl;
	thrust::device_vector<int> D = H;
	D[0] = 99;
	D[1] = 88;

	for (int i = 0; i < D.size(); i++)
		std::cout << "D[" << i << "] = " << D[i] << std::endl;
}

void test_Thrust_v2()
{
	// initialize all ten integers of a device_vector to 1
	thrust::device_vector<int> D(10, 1);
	// set the first seven elements of a vector to 9
	thrust::fill(D.begin(), D.begin() + 7, 9);
	// initialize a host_vector with the first five elements of D
	thrust::host_vector<int> H(D.begin(), D.begin() + 5);
	// set the elements of H to 0, 1, 2, 3, ...
	thrust::sequence(H.begin(), H.end());
	// copy all of H back to the beginning of D
	thrust::copy(H.begin(), H.end(), D.begin());
	// print D
	for (int i = 0; i < D.size(); i++)
		std::cout << "D[" << i << "] = " << D[i] << std::endl;
}

void test_Thrust_v3()
{
	// generate 16M random numbers on the host
	thrust::host_vector<int> h_vec(1 << 8);
	thrust::generate(h_vec.begin(), h_vec.end(), rand);
	// transfer data to the device
	thrust::device_vector<int> d_vec = h_vec;
	// sort data on the device
	thrust::sort(d_vec.begin(), d_vec.end());
	// transfer data back to host
	thrust::copy(d_vec.begin(), d_vec.end(), h_vec.begin());

	for (int i = 0; i < d_vec.size(); i++)
		std::cout << "D[" << i << "] = " << d_vec[i] << std::endl;
}


__global__ void my_kernel(int * a)
{
	int i = threadIdx.x + blockIdx.x*blockDim.x;
	a[i] = i;
}

void test_Thrust_toCUDA()
{
	size_t N = 256;
	int threadsPerBlock = 128;
	int blocksPerGrid = 2;

	/*thrust::device_ptr<int> dev_ptr = thrust::device_malloc<int>(N);
	int * raw_ptr_x = thrust::raw_pointer_cast(dev_ptr);
	cudaMemset(raw_ptr_x, 0, N * sizeof(int));
	thrust::device_free(dev_ptr);*/

	thrust::device_vector<int> d_vec(N);

	// note: d_vec.data() returns a device_ptr
	int * raw_ptr = thrust::raw_pointer_cast(d_vec.data());

	//cudaMemset(raw_ptr, 10, N * sizeof(int));
	
	my_kernel << < blocksPerGrid, threadsPerBlock >> >(raw_ptr);

	for (int i = 0; i < d_vec.size(); i++)
		std::cout << "D[" << i << "] = " << d_vec[i] << std::endl;
}

void test_Thrust_fromCUDA()
{
	size_t N = 10;

	// obtain raw pointer to device memory
	int * raw_ptr;
	cudaMalloc((void **)&raw_ptr, N * sizeof(int));

	// wrap raw pointer with a device_ptr 
	thrust::device_ptr<int> dev_ptr = thrust::device_pointer_cast(raw_ptr);

	// use device_ptr in Thrust algorithms
	thrust::fill(dev_ptr, dev_ptr + N, (int)0);

	// access device memory transparently through device_ptr
	dev_ptr[0] = 1;

	// free memory
	cudaFree(raw_ptr);
}
