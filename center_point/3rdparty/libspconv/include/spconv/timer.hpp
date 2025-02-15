#ifndef __SPCONV_TIMER_HPP__
#define __SPCONV_TIMER_HPP__

#include <cuda_runtime.h>

#include "check.hpp"

namespace spconv {

class EventTimer {
 public:
  EventTimer() {
    checkRuntime(cudaEventCreate(&begin_));
    checkRuntime(cudaEventCreate(&end_));
  }

  virtual ~EventTimer() {
    checkRuntime(cudaEventDestroy(begin_));
    checkRuntime(cudaEventDestroy(end_));
  }

  void start(void *stream) {
    stream_ = (cudaStream_t)stream;
    checkRuntime(cudaEventRecord(begin_, (cudaStream_t)stream));
  }

  float stop(const char *prefix, bool print = true) {
    float times = 0;
    checkRuntime(cudaEventRecord(end_, stream_));
    checkRuntime(cudaEventSynchronize(end_));
    checkRuntime(cudaEventElapsedTime(&times, begin_, end_));
    if (print) printf("[Times %s]: %.3f ms\n", prefix, times);
    return times;
  }

 private:
  cudaStream_t stream_ = nullptr;
  cudaEvent_t begin_ = nullptr, end_ = nullptr;
};

};  // namespace spconv

#endif  // #ifndef __SPCONV_TIMER_HPP__