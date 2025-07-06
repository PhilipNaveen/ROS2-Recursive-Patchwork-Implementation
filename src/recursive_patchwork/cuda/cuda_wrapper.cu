#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/transform.h>
#include <thrust/remove.h>
#include <thrust/copy.h>
#include <thrust/functional.h>
#include <thrust/tuple.h>
#include <vector>
#include <cstdio>

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

// Phase 1: Distance calculations kernel
__global__ void computeDistances2DKernel(float* x, float* y, float* distances, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        float dx = x[idx];
        float dy = y[idx];
        distances[idx] = sqrtf(dx * dx + dy * dy);
    }
}

// Phase 1: Point filtering kernel (filter by radius)
__global__ void filterPointsByRadiusKernel(float* x, float* y, float* z, float* distances,
                                          bool* mask, float radius, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        mask[idx] = (distances[idx] <= radius);
    }
}

// Phase 1: Angle calculations kernel
__global__ void computeAnglesKernel(float* x, float* y, float* angles, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        float angle = atan2f(y[idx], x[idx]);
        if (angle < 0) angle += 2.0f * M_PI;
        angles[idx] = angle;
    }
}

// Phase 1: Patch classification kernel
__global__ void classifyPointsInPatchKernel(float* distances, float* angles,
                                           bool* mask, int n,
                                           float r0, float r1, float a0, float a1) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        bool in_radius = (distances[idx] >= r0 && distances[idx] < r1);
        bool in_angle = (angles[idx] >= a0 && angles[idx] < a1);
        mask[idx] = in_radius && in_angle;
    }
}

// Phase 1: Plane distance computations kernel
__global__ void computePlaneDistancesKernel(float* x, float* y, float* z,
                                           float* distances, int n,
                                           float centroid_x, float centroid_y, float centroid_z,
                                           float normal_x, float normal_y, float normal_z) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        float dx = x[idx] - centroid_x;
        float dy = y[idx] - centroid_y;
        float dz = z[idx] - centroid_z;
        
        float dot_product = dx * normal_x + dy * normal_y + dz * normal_z;
        distances[idx] = fabsf(dot_product);
    }
}

// Phase 1: Ground point classification kernel
__global__ void classifyGroundPointsKernel(float* plane_distances, bool* ground_mask,
                                          float threshold, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        ground_mask[idx] = (plane_distances[idx] < threshold);
    }
}

// Wrapper functions that can be called from C++
void cuda_rotate_points(float* d_x, float* d_y, float* d_z, 
                       float cos_a, float sin_a, int n) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    rotatePointsKernel<<<numBlocks, blockSize>>>(d_x, d_y, d_z, cos_a, sin_a, n);
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA error in rotatePointsKernel: %s\n", cudaGetErrorString(error));
    }
    cudaDeviceSynchronize();
}

void cuda_transform_points(float* d_x, float* d_y, float* d_z,
                          float* d_matrix, int n) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    transformPointsKernel<<<numBlocks, blockSize>>>(d_x, d_y, d_z, d_matrix, n);
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA error in transformPointsKernel: %s\n", cudaGetErrorString(error));
    }
    cudaDeviceSynchronize();
}

// Phase 1 wrapper functions
void cuda_compute_distances_2d(float* d_x, float* d_y, float* d_distances, int n) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    computeDistances2DKernel<<<numBlocks, blockSize>>>(d_x, d_y, d_distances, n);
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA error in computeDistances2DKernel: %s\n", cudaGetErrorString(error));
    }
    cudaDeviceSynchronize();
}

void cuda_filter_points_by_radius(float* d_x, float* d_y, float* d_z, float* d_distances,
                                 bool* d_mask, float radius, int n) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    filterPointsByRadiusKernel<<<numBlocks, blockSize>>>(d_x, d_y, d_z, d_distances, d_mask, radius, n);
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA error in filterPointsByRadiusKernel: %s\n", cudaGetErrorString(error));
    }
    cudaDeviceSynchronize();
}

void cuda_compute_angles(float* d_x, float* d_y, float* d_angles, int n) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    computeAnglesKernel<<<numBlocks, blockSize>>>(d_x, d_y, d_angles, n);
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA error in computeAnglesKernel: %s\n", cudaGetErrorString(error));
    }
    cudaDeviceSynchronize();
}

void cuda_classify_points_in_patch(float* d_distances, float* d_angles,
                                  bool* d_mask, int n,
                                  float r0, float r1, float a0, float a1) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    classifyPointsInPatchKernel<<<numBlocks, blockSize>>>(d_distances, d_angles, d_mask, n, r0, r1, a0, a1);
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA error in classifyPointsInPatchKernel: %s\n", cudaGetErrorString(error));
    }
    cudaDeviceSynchronize();
}

void cuda_compute_plane_distances(float* d_x, float* d_y, float* d_z,
                                 float* d_distances, int n,
                                 float centroid_x, float centroid_y, float centroid_z,
                                 float normal_x, float normal_y, float normal_z) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    computePlaneDistancesKernel<<<numBlocks, blockSize>>>(d_x, d_y, d_z, d_distances, n,
                                                         centroid_x, centroid_y, centroid_z,
                                                         normal_x, normal_y, normal_z);
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA error in computePlaneDistancesKernel: %s\n", cudaGetErrorString(error));
    }
    cudaDeviceSynchronize();
}

void cuda_classify_ground_points(float* d_plane_distances, bool* d_ground_mask,
                                float threshold, int n) {
    int blockSize = 256;
    int numBlocks = (n + blockSize - 1) / blockSize;
    classifyGroundPointsKernel<<<numBlocks, blockSize>>>(d_plane_distances, d_ground_mask, threshold, n);
    cudaError_t error = cudaGetLastError();
    if (error != cudaSuccess) {
        printf("CUDA error in classifyGroundPointsKernel: %s\n", cudaGetErrorString(error));
    }
    cudaDeviceSynchronize();
} 