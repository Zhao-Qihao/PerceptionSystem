#include <memory>

#include "common.h"
#include "NvInfer.h"
#include "NvOnnxConfig.h"
#include "NvInferRuntime.h"
#include "preprocess.h"
#include "postprocess.h"
#include "spconv/engine.hpp"
#include "tensorrt.hpp"
#include "timer.hpp"

typedef struct float11 { float val[11]; } float11;

class CenterPoint {
  private:
    Params params_;
    bool verbose_;

    std::shared_ptr<PreProcessCuda> pre_;
    std::shared_ptr<spconv::Engine> scn_engine_;
    std::shared_ptr<TensorRT::Engine> trt_;
    std::shared_ptr<PostProcessCuda> post_;

    std::vector<float> timing_pre_;
    std::vector<float> timing_scn_engine_;
    std::vector<float> timing_trt_;
    std::vector<float> timing_post_;

    unsigned int* h_detections_num_;
    float* d_detections_;
    float* d_detections_reshape_;     //add d_detections_reshape_

    half* d_reg_[NUM_TASKS];
    half* d_height_[NUM_TASKS];
    half* d_dim_[NUM_TASKS];
    half* d_rot_[NUM_TASKS];
    half* d_vel_[NUM_TASKS];
    half* d_hm_[NUM_TASKS];

    int reg_n_;
    int reg_c_;
    int reg_h_;
    int reg_w_;
    int height_c_;
    int dim_c_;
    int rot_c_;
    int vel_c_;
    int hm_c_[NUM_TASKS];

    half* d_voxel_features;
    unsigned int* d_voxel_indices;
    std::vector<int> sparse_shape;

    std::vector<float11> detections_;
    unsigned int h_mask_size_;
    uint64_t* h_mask_ = nullptr;
    EventTimer timer_;
    std::unordered_map<int, std::string> label_map_;

  public:
    CenterPoint(std::string packagePath, std::string modelFile, bool verbose = false);
    ~CenterPoint(void);

    int prepare();
    int doinfer(void* points, unsigned int point_num, cudaStream_t stream);
    std::vector<Bndbox> nms_pred_;
    void perf_report();
};