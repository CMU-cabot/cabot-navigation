#ifndef CABOT_DNN_CONTROLLER__TENSORRT_UTILS_HPP_
#define CABOT_DNN_CONTROLLER__TENSORRT_UTILS_HPP_

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <stdexcept>
#include <string>
#include <vector>

namespace cabot_dnn_controller
{
namespace tensorrt_utils
{

#define CUDA_CHECK(expr) \
  do { \
    cudaError_t _e = (expr); \
    if (_e != cudaSuccess) { \
      throw std::runtime_error(std::string("CUDA error: ") + cudaGetErrorString(_e)); \
    } \
  } while (0)

inline size_t volume(const nvinfer1::Dims & d)
{
  size_t v = 1;
  for (int i = 0; i < d.nbDims; ++i) {
    if (d.d[i] < 0) {
      throw std::runtime_error("TensorRT Dims should be positive");
    }
    v *= static_cast<size_t>(d.d[i]);
  }
  return v;
}

inline size_t elementSize(nvinfer1::DataType t)
{
  switch (t) {
    case nvinfer1::DataType::kFLOAT: return 4;
    case nvinfer1::DataType::kHALF:  return 2;
    case nvinfer1::DataType::kINT8:  return 1;
    case nvinfer1::DataType::kINT32: return 4;
    case nvinfer1::DataType::kBOOL:  return 1;
    default: throw std::runtime_error("Unsupported TensorRT DataType");
  }
}

inline size_t bytesFor(const nvinfer1::Dims & d, nvinfer1::DataType t)
{
  return volume(d) * elementSize(t);
}

inline void copyFloatHostToDevice(
  void * dst, const float * src, size_t count, nvinfer1::DataType dt, cudaStream_t stream)
{
  if (dt == nvinfer1::DataType::kFLOAT) {
    CUDA_CHECK(cudaMemcpyAsync(dst, src, count * sizeof(float), cudaMemcpyHostToDevice, stream));
    return;
  }
  if (dt == nvinfer1::DataType::kHALF) {
    std::vector<__half> tmp(count);
    for (size_t i = 0; i < count; ++i) {
      tmp[i] = __float2half_rn(src[i]);
    }
    CUDA_CHECK(cudaMemcpyAsync(dst, tmp.data(), count * sizeof(__half), cudaMemcpyHostToDevice, stream));
    return;
  }
  throw std::runtime_error("Unsupported TensorRT input data type");
}

inline void copyDeviceToHostFloat(
  float * dst, const void * src, size_t count, nvinfer1::DataType dt, cudaStream_t stream)
{
  if (dt == nvinfer1::DataType::kFLOAT) {
    CUDA_CHECK(cudaMemcpyAsync(dst, src, count * sizeof(float), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    return;
  }
  if (dt == nvinfer1::DataType::kHALF) {
    std::vector<__half> tmp(count);
    CUDA_CHECK(cudaMemcpyAsync(tmp.data(), src, count * sizeof(__half), cudaMemcpyDeviceToHost, stream));
    CUDA_CHECK(cudaStreamSynchronize(stream));
    for (size_t i = 0; i < count; ++i) {
      dst[i] = __half2float(tmp[i]);
    }
    return;
  }
  throw std::runtime_error("Unsupported TensorRT output data type");
}

}  // namespace tensorrt_utils
}  // namespace cabot_dnn_controller

#endif  // CABOT_DNN_CONTROLLER__TENSORRT_UTILS_HPP_
