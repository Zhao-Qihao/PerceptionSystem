
#include "postprocess.h"
#include <iostream>
#include <vector>

PostProcessCuda::PostProcessCuda()
{
  checkCudaErrors(cudaMalloc((void **)&d_post_center_range_, 6 * sizeof(float)));
  checkCudaErrors(cudaMalloc((void **)&d_voxel_size_, 2 * sizeof(float)));
  checkCudaErrors(cudaMalloc((void **)&d_pc_range_, 2 * sizeof(float)));

  checkCudaErrors(cudaMemcpy(d_post_center_range_, params_.post_center_range, 6 * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(d_voxel_size_, params_.voxel_size, 2 * sizeof(float), cudaMemcpyHostToDevice));
  checkCudaErrors(cudaMemcpy(d_pc_range_, params_.pc_range, 2 * sizeof(float), cudaMemcpyHostToDevice));
  return;
}

PostProcessCuda::~PostProcessCuda()
{
  checkCudaErrors(cudaFree(d_post_center_range_));
  checkCudaErrors(cudaFree(d_voxel_size_));
  checkCudaErrors(cudaFree(d_pc_range_));
  return;
}

int PostProcessCuda::doPostDecodeCuda(
    int N,
    int H,
    int W,
    int C_reg,
    int C_height,
    int C_dim,
    int C_rot,
    int C_vel,
    int C_hm,
    const half *reg,
    const half *height,
    const half *dim,
    const half *rot,
    const half *vel,
    const half *hm,
    unsigned int *detection_num,
    float *detections, cudaStream_t stream)
{
    postprocess_launch(
                      N,
                      H,
                      W,
                      C_reg,
                      C_height,
                      C_dim,
                      C_rot,
                      C_vel,
                      C_hm,
                      reg,
                      height,
                      dim,
                      rot,
                      vel,
                      hm,
                      detection_num,
                      detections,
                      d_post_center_range_,
                      params_.out_size_factor,
                      d_voxel_size_,
                      d_pc_range_,
                      params_.score_threshold,
                      stream
                      );
  return 0;
}

int PostProcessCuda::doPostNMSCuda(
    unsigned int boxes_num,
    float *boxes_sorted,
    uint64_t* mask, cudaStream_t stream)
{
  if(boxes_num > params_.nms_pre_max_size){
      std::cerr << "[ERR] Boxs num exceeds:" << params_.nms_pre_max_size << std::endl;
      // exit(-1);
  }

  nms_launch(boxes_num, boxes_sorted, params_.nms_iou_threshold, mask, stream);
  return 0;
}

int PostProcessCuda::doPermuteCuda(
    unsigned int boxes_num, 
    const float *boxes_sorted, 
    float * permute_boxes, cudaStream_t stream)
{ 

  if(boxes_num > params_.nms_pre_max_size){
      std::cerr << "[ERR] Boxs num exceeds:" << params_.nms_pre_max_size << std::endl;
      // exit(-1);
  }

  permute_launch(boxes_num, boxes_sorted, permute_boxes, stream);
  return 0;
}
