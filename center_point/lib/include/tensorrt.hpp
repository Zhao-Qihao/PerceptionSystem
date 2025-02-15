
#ifndef TENSORRT_HPP
#define TENSORRT_HPP

#include <string>
#include <memory>
#include <vector>

namespace TensorRT{

    class Engine{
    public:
        virtual int64_t getBindingNumel(const std::string& name) = 0;
        virtual std::vector<int64_t> getBindingDims(const std::string& name) = 0;
        virtual bool forward(const std::initializer_list<void*>& buffers, void* stream = nullptr) = 0;
        virtual void print() = 0;
    };

    std::shared_ptr<Engine> load(const std::string& file);
};

#endif // TENSORRT_HPP