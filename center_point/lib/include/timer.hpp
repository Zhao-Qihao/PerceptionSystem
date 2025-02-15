
#ifndef TIMER_HPP
#define TIMER_HPP

#include "common.h"

class EventTimer{
public:
    EventTimer(){
        checkCudaErrors(cudaEventCreate(&begin_));
        checkCudaErrors(cudaEventCreate(&end_));
    }

    virtual ~EventTimer(){
        checkCudaErrors(cudaEventDestroy(begin_));
        checkCudaErrors(cudaEventDestroy(end_));
    }

    void start(cudaStream_t stream){
        checkCudaErrors(cudaEventRecord(begin_, stream));
    }

    float stop(const char* prefix = "timer", bool print = true){
        float times = 0;
        checkCudaErrors(cudaEventRecord(end_, stream_));
        checkCudaErrors(cudaEventSynchronize(end_));
        checkCudaErrors(cudaEventElapsedTime(&times, begin_, end_));
        if(print) printf("[TIME] %s:\t\t%.5f ms\n", prefix, times);
        return times;
    }

private:
    cudaStream_t stream_ = nullptr;
    cudaEvent_t begin_ = nullptr, end_ = nullptr;
};




#endif // TIMER_HPP