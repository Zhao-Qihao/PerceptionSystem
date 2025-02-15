
#include "kernel.h"

class PreProcessCuda {
  private:
    Params params_;
    unsigned int *point2voxel_offset_;
    unsigned int *hash_table_;
    float *voxels_temp_;

    unsigned int *d_real_num_voxels_;
    unsigned int *h_real_num_voxels_;
    half *d_voxel_features_;
    unsigned int *d_voxel_num_;
    unsigned int *d_voxel_indices_;

    unsigned int hash_table_size_;
    unsigned int voxels_temp_size_;
    unsigned int voxel_features_size_;
    unsigned int voxel_idxs_size_;
    unsigned int voxel_num_size_;

  public:
    PreProcessCuda();
    ~PreProcessCuda();

    int alloc_resource();
    int generateVoxels(const float *points, size_t points_size, cudaStream_t stream);
    unsigned int getOutput(half** d_voxel_features, unsigned int** d_voxel_indices, std::vector<int>& sparse_shape);
};