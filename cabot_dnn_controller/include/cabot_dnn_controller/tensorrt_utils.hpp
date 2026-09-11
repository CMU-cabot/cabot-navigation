#ifndef CABOT_DNN_CONTROLLER__TENSORRT_UTILS_HPP_
#define CABOT_DNN_CONTROLLER__TENSORRT_UTILS_HPP_

#include <NvInfer.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>
#include <yaml-cpp/yaml.h>

#include <cstdint>
#include <initializer_list>
#include <stdexcept>
#include <string>
#include <vector>

#include "cabot_dnn_controller/dnn_controller_constants.hpp"

namespace cabot_dnn_controller
{
namespace tensorrt_utils
{

inline int readActionLength(const YAML::Node & action_config)
{
  const int action_length = action_config["action_length"] ?
    action_config["action_length"].as<int>() : 10;
  if (action_length <= 0) {
    throw std::runtime_error("action.action_length must be a positive integer");
  }
  return action_length;
}

inline void validateActionOutputShapes(
  const nvinfer1::Dims & cmd_dims, const nvinfer1::Dims & v_logits_dims,
  const nvinfer1::Dims & w_logits_dims, const YAML::Node & action_config,
  dnn_controller_constants::ActionMode mode, int action_length)
{
  using dnn_controller_constants::ActionMode;
  const auto validate = [](const nvinfer1::Dims & dims, const char * name,
      std::initializer_list<std::int64_t> expected) {
      bool matches = dims.nbDims == static_cast<int>(expected.size());
      int i = 0;
      for (const auto dimension : expected) {
        matches = matches && dimension > 0 && dims.d[i] == dimension;
        ++i;
      }
      if (!matches) {
        throw std::runtime_error(
                std::string("TensorRT ") + name +
                " output shape does not match action config; expected a batch and "
                "action_length axis, including action_length=1. Re-export the model.");
      }
    };

  std::int64_t v_width = 1;
  std::int64_t w_width = 1;
  if (mode == ActionMode::kCls || mode == ActionMode::kClsReg) {
    v_width = action_config["v_num_bins"].as<int>();
    w_width = action_config["w_num_bins"].as<int>();
  } else if (mode == ActionMode::kMdnReg) {
    v_width = 3 * static_cast<std::int64_t>(action_config["v_k"].as<int>());
    w_width = 3 * static_cast<std::int64_t>(action_config["w_k"].as<int>());
  }
  if (mode == ActionMode::kClsReg) {
    validate(cmd_dims, dnn_controller_constants::kOutputCmdName,
      {1, action_length, v_width * w_width, 2});
  } else {
    validate(cmd_dims, dnn_controller_constants::kOutputCmdName, {1, action_length, 2});
  }
  validate(v_logits_dims, dnn_controller_constants::kOutputVLogitsName,
    {1, action_length, v_width});
  validate(w_logits_dims, dnn_controller_constants::kOutputWLogitsName,
    {1, action_length, w_width});
}

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
