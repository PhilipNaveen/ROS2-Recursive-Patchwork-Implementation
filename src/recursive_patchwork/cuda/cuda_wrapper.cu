#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/remove.h>
#include <thrust/copy.h>
#include <thrust/functional.h>
#include <thrust/tuple.h>
#include <vector>

// Include the kernels
#include "cuda_kernels.cu"

// Forward declarations
extern "C" {
    // Wrapper functions that can be called from C++
    void cuda_rotate_points(float* d_x, float* d_y, float* d_z, 
                           float cos_a, float sin_a, int n);
    void cuda_transform_points(float* d_x, float* d_y, float* d_z,
                              float* d_matrix, int n);
}

// CUDA wrapper for rotation kernel
void cuda_rotate_points(float* d_x, float* d_y, float* d_z, 
                       float cos_a, float sin_a, int n) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    rotatePointsKernel<<<numBlocks, blockSize>>>(d_x, d_y, d_z, cos_a, sin_a, n);
    cudaDeviceSynchronize();
}

// CUDA wrapper for transformation kernel
void cuda_transform_points(float* d_x, float* d_y, float* d_z,
                          float* d_matrix, int n) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    transformPointsKernel<<<numBlocks, blockSize>>>(d_x, d_y, d_z, d_matrix, n);
    cudaDeviceSynchronize();
} 