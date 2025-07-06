#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/remove.h>
#include <thrust/copy.h>
#include <thrust/functional.h>
#include <thrust/tuple.h>

// CUDA kernel for 2D rotation
__global__ void rotatePointsKernel(float* x, float* y, float* z, 
                                  float cos_a, float sin_a, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        float x_orig = x[idx];
        float y_orig = y[idx];
        x[idx] = x_orig * cos_a - y_orig * sin_a;
        y[idx] = x_orig * sin_a + y_orig * cos_a;
        // z remains unchanged
    }
}

// CUDA kernel for 4x4 transformation
__global__ void transformPointsKernel(float* x, float* y, float* z,
                                     float* matrix, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        float x_orig = x[idx];
        float y_orig = y[idx];
        float z_orig = z[idx];
        
        // Apply 4x4 transformation matrix
        float x_new = matrix[0] * x_orig + matrix[1] * y_orig + matrix[2] * z_orig + matrix[3];
        float y_new = matrix[4] * x_orig + matrix[5] * y_orig + matrix[6] * z_orig + matrix[7];
        float z_new = matrix[8] * x_orig + matrix[9] * y_orig + matrix[10] * z_orig + matrix[11];
        float w = matrix[12] * x_orig + matrix[13] * y_orig + matrix[14] * z_orig + matrix[15];
        
        // Homogeneous division
        x[idx] = x_new / w;
        y[idx] = y_new / w;
        z[idx] = z_new / w;
    }
}

// Functor for ego vehicle filtering
struct EgoVehicleFilter {
    float radius_squared;
    
    EgoVehicleFilter(float radius) : radius_squared(radius * radius) {}
    
    __device__ bool operator()(const thrust::tuple<float, float, float>& point) const {
        float x = thrust::get<0>(point);
        float y = thrust::get<1>(point);
        float distance_squared = x * x + y * y;
        return distance_squared > radius_squared;
    }
}; 