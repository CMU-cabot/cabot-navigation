#include <NvInfer.h>
#include <NvInferPlugin.h>
#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include "cabot_dnn_controller/classify_utils.hpp"
#include "cabot_dnn_controller/dnn_controller_constants.hpp"
#include "cabot_dnn_controller/tensorrt_utils.hpp"

#include <filesystem>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace
{

using cabot_dnn_controller::dnn_controller_constants::ActionMode;

using cabot_dnn_controller::dnn_controller_constants::kScanLength;
using cabot_dnn_controller::dnn_controller_constants::kPeopleDim;

using cabot_dnn_controller::dnn_controller_constants::kVMin;
using cabot_dnn_controller::dnn_controller_constants::kVMax;
using cabot_dnn_controller::dnn_controller_constants::kWMin;
using cabot_dnn_controller::dnn_controller_constants::kWMax;

using cabot_dnn_controller::dnn_controller_constants::kInputOdomName;
using cabot_dnn_controller::dnn_controller_constants::kInputPlanName;
using cabot_dnn_controller::dnn_controller_constants::kInputScanName;
using cabot_dnn_controller::dnn_controller_constants::kInputPeopleName;
using cabot_dnn_controller::dnn_controller_constants::kOutputCmdName;
using cabot_dnn_controller::dnn_controller_constants::kOutputVLogitsName;
using cabot_dnn_controller::dnn_controller_constants::kOutputWLogitsName;
using cabot_dnn_controller::dnn_controller_constants::kOutputPeopleAttentionName;
using cabot_dnn_controller::dnn_controller_constants::kOutputRobotPeopleAttentionName;

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity == Severity::kVERBOSE) {
      return;
    }
    std::cerr << "[TensorRT] " << msg << std::endl;
  }
};

static bool hasIOTensor(const nvinfer1::ICudaEngine & engine, const char * tensor_name)
{
  for (int i = 0; i < engine.getNbIOTensors(); ++i) {
    if (std::string(engine.getIOTensorName(i)) == tensor_name) {
      return true;
    }
  }
  return false;
}

static std::string dimsToString(const nvinfer1::Dims & dims)
{
  std::ostringstream oss;
  oss << "(";
  for (int i = 0; i < dims.nbDims; ++i) {
    if (i > 0) {
      oss << ", ";
    }
    oss << dims.d[i];
  }
  oss << ")";
  return oss.str();
}

static std::vector<float> readFloatBin(const std::filesystem::path & path, size_t expected_count)
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) {
    throw std::runtime_error("Failed to open: " + path.string());
  }
  ifs.seekg(0, std::ios::end);
  const std::streamoff size = ifs.tellg();
  if (size < 0) {
    throw std::runtime_error("Failed to read file size: " + path.string());
  }
  ifs.seekg(0, std::ios::beg);
  if (static_cast<size_t>(size) % sizeof(float) != 0) {
    throw std::runtime_error("File size is not aligned to float32: " + path.string());
  }
  const size_t count = static_cast<size_t>(size) / sizeof(float);
  std::vector<float> data(count);
  ifs.read(reinterpret_cast<char *>(data.data()), static_cast<std::streamsize>(size));
  if (expected_count != 0 && count != expected_count) {
    throw std::runtime_error(
      "Unexpected element count in " + path.string() + ": got " + std::to_string(count) +
      ", expected " + std::to_string(expected_count));
  }
  return data;
}

static void printFloatVector(const std::string & label, const std::vector<float> & values)
{
  std::cout << label << ": [";
  for (size_t i = 0; i < values.size(); ++i) {
    if (i > 0) {
      std::cout << ", ";
    }
    std::cout << std::setprecision(8) << values[i];
  }
  std::cout << "]" << std::endl;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    std::string model_file;
    std::string data_dir;
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      if ((arg == "-m" || arg == "--model-file") && i + 1 < argc) {
        model_file = argv[++i];
      } else if ((arg == "-d" || arg == "--data-dir") && i + 1 < argc) {
        data_dir = argv[++i];
      }
    }
    if (model_file.empty() || data_dir.empty()) {
      std::cerr << "Usage: " << argv[0] << " -m <model_file> -d <data_dir>" << std::endl;
      return 1;
    }

    const std::filesystem::path model_path(model_file);
    if (!std::filesystem::exists(model_path)) {
      std::cerr << "TensorRT model file not found: " << model_path << std::endl;
      return 1;
    }

    const std::filesystem::path config_path = model_path.parent_path() / "config.yaml";
    if (!std::filesystem::exists(config_path)) {
      std::cerr << "Model config file not found: " << config_path << std::endl;
      return 1;
    }
    const YAML::Node config = YAML::LoadFile(config_path.string());
    int odom_length = config["odom_encoder"]["odom_length"].as<int>();
    int plan_length = config["plan_encoder"]["plan_length"].as<int>();
    bool people_encoder_enabled = false;
    int num_people = 0;
    int people_history_length = 0;
    int num_attention_heads = 0;
    const YAML::Node people_config = config["people_encoder"];
    if (people_config && people_config["enabled"]) {
      people_encoder_enabled = people_config["enabled"].as<bool>();
    }
    if (people_encoder_enabled) {
      if (!people_config["num_people"]) {
        std::cerr << "people_encoder.num_people is required when people_encoder is enabled" << std::endl;
        return 1;
      }
      num_people = people_config["num_people"].as<int>();
      if (num_people <= 0) {
        std::cerr << "people_encoder.num_people must be > 0" << std::endl;
        return 1;
      }
      if (!people_config["history_length"]) {
        std::cerr << "people_encoder.history_length is required when people_encoder is enabled" << std::endl;
        return 1;
      }
      people_history_length = people_config["history_length"].as<int>();
      if (people_history_length <= 0) {
        std::cerr << "people_encoder.history_length must be > 0" << std::endl;
        return 1;
      }
      num_attention_heads = people_config["num_attention_heads"].as<int>();
      if (num_attention_heads <= 0) {
        std::cerr << "people_encoder.num_attention_heads must be > 0" << std::endl;
        return 1;
      }
    }
    std::string action_mode_str = config["action"]["mode"].as<std::string>();
    ActionMode action_mode;
    if (action_mode_str == "reg") {
      action_mode = ActionMode::kReg;
    } else if (action_mode_str == "cls") {
      action_mode = ActionMode::kCls;
    } else if (action_mode_str == "cls-reg") {
      action_mode = ActionMode::kClsReg;
    } else if (action_mode_str == "mdn-reg") {
      action_mode = ActionMode::kMdnReg;
    } else {
      std::cerr << "Invalid action mode: " << action_mode_str << std::endl;
      return 1;
    }
    int v_num_bins;
    int w_num_bins;
    if ((action_mode == ActionMode::kCls) || (action_mode == ActionMode::kClsReg)) {
      v_num_bins = config["action"]["v_num_bins"].as<int>();
      w_num_bins = config["action"]["w_num_bins"].as<int>();
    }

    const std::filesystem::path data_path(data_dir);
    const std::filesystem::path odom_path = data_path / "odom.bin";
    const std::filesystem::path plan_path = data_path / "plan.bin";
    const std::filesystem::path scan_path = data_path / "scan.bin";
    const std::filesystem::path people_path = data_path / "people.bin";
    const std::filesystem::path out_path = data_path / "out.bin";
    const std::filesystem::path people_attention_path = data_path / "people_attention.bin";
    const std::filesystem::path robot_people_attention_path =
      data_path / "robot_people_attention.bin";

    const size_t odom_count = static_cast<size_t>(odom_length) * 2;
    const size_t plan_count = static_cast<size_t>(plan_length) * 2;
    const size_t scan_count = static_cast<size_t>(kScanLength);
    const size_t people_count =
      static_cast<size_t>(num_people) * static_cast<size_t>(people_history_length) * kPeopleDim;

    const std::vector<float> h_odom = readFloatBin(odom_path, odom_count);
    const std::vector<float> h_plan = readFloatBin(plan_path, plan_count);
    const std::vector<float> h_scan = readFloatBin(scan_path, scan_count);
    std::vector<float> h_people;
    std::vector<float> h_people_attention_gt;
    std::vector<float> h_robot_people_attention_gt;
    if (people_encoder_enabled) {
      h_people = readFloatBin(people_path, people_count);
      const size_t attention_head_count = static_cast<size_t>(num_attention_heads);
      const size_t attention_person_count = static_cast<size_t>(num_people);
      h_people_attention_gt = readFloatBin(
        people_attention_path,
        attention_head_count * attention_person_count * attention_person_count);
      h_robot_people_attention_gt = readFloatBin(
        robot_people_attention_path, attention_head_count * attention_person_count);
    }
    const std::vector<float> h_out_gt = readFloatBin(out_path, 2);

    TrtLogger logger;
    if (!initLibNvInferPlugins(&logger, "")) {
      std::cerr << "Failed to initialize TensorRT plugins" << std::endl;
      return 1;
    }

    std::unique_ptr<nvinfer1::IRuntime, void(*)(nvinfer1::IRuntime*)> runtime(
      nvinfer1::createInferRuntime(logger),
      [](nvinfer1::IRuntime * p){ if (p) { delete p; } });
    if (!runtime) {
      std::cerr << "Failed to create TensorRT runtime" << std::endl;
      return 1;
    }

    std::ifstream ifs(model_path, std::ios::binary);
    if (!ifs) {
      std::cerr << "Failed to open engine file: " << model_path << std::endl;
      return 1;
    }
    std::vector<char> engine_data((std::istreambuf_iterator<char>(ifs)), {});
    if (engine_data.empty()) {
      std::cerr << "Engine file is empty: " << model_path << std::endl;
      return 1;
    }

    std::unique_ptr<nvinfer1::ICudaEngine, void(*)(nvinfer1::ICudaEngine*)> engine(
      runtime->deserializeCudaEngine(engine_data.data(), engine_data.size()),
      [](nvinfer1::ICudaEngine * p){ if (p) { delete p; } });
    if (!engine) {
      std::cerr << "Failed to deserialize TensorRT engine: " << model_path << std::endl;
      return 1;
    }
    const bool has_people_tensor = hasIOTensor(*engine, kInputPeopleName);
    if (people_encoder_enabled && !has_people_tensor) {
      std::cerr << "people_encoder is enabled, but TensorRT engine has no '" << kInputPeopleName <<
        "' input tensor" << std::endl;
      return 1;
    }
    if (people_encoder_enabled &&
      (!hasIOTensor(*engine, kOutputPeopleAttentionName) ||
      !hasIOTensor(*engine, kOutputRobotPeopleAttentionName)))
    {
      std::cerr << "people_encoder is enabled, but TensorRT engine has no attention outputs; "
                << "re-export the engine" << std::endl;
      return 1;
    }
    if (!people_encoder_enabled && has_people_tensor) {
      std::cerr << "TensorRT engine has a '" << kInputPeopleName <<
        "' input tensor, but people_encoder is disabled in config.yaml" << std::endl;
      return 1;
    }

    std::unique_ptr<nvinfer1::IExecutionContext, void(*)(nvinfer1::IExecutionContext*)> context(
      engine->createExecutionContext(),
      [](nvinfer1::IExecutionContext * p){ if (p) { delete p; } });
    if (!context) {
      std::cerr << "Failed to create TensorRT execution context" << std::endl;
      return 1;
    }

    bool input_shapes_set =
      context->setInputShape(kInputOdomName, nvinfer1::Dims3{1, odom_length, 2}) &&
      context->setInputShape(kInputPlanName, nvinfer1::Dims3{1, plan_length, 2}) &&
      context->setInputShape(kInputScanName, nvinfer1::Dims2{1, kScanLength});
    if (people_encoder_enabled) {
      input_shapes_set = input_shapes_set &&
        context->setInputShape(
          kInputPeopleName,
          nvinfer1::Dims4{1, num_people, people_history_length, kPeopleDim});
    }
    if (!input_shapes_set) {
      std::cerr << "Failed to set input shapes" << std::endl;
      return 1;
    }
    if (people_encoder_enabled) {
      const nvinfer1::Dims people_dims = context->getTensorShape(kInputPeopleName);
      if (people_dims.nbDims != 4 || people_dims.d[0] != 1 ||
        people_dims.d[1] != num_people || people_dims.d[2] != people_history_length ||
        people_dims.d[3] != kPeopleDim)
      {
        std::cerr << "Unexpected people input shape: got " << dimsToString(people_dims) <<
          ", expected (1, " << num_people << ", " << people_history_length << ", " <<
          kPeopleDim << ")" << std::endl;
        return 1;
      }
    }

    cudaStream_t stream = nullptr;
    CUDA_CHECK(cudaStreamCreate(&stream));

    std::unordered_map<std::string, void*> device_buffers;
    const int nb_io = engine->getNbIOTensors();
    for (int i = 0; i < nb_io; ++i) {
      const char * name = engine->getIOTensorName(i);
      const nvinfer1::Dims dims = context->getTensorShape(name);
      const nvinfer1::DataType dtype = engine->getTensorDataType(name);
      const size_t nbytes = cabot_dnn_controller::tensorrt_utils::bytesFor(dims, dtype);
      void * ptr = nullptr;
      CUDA_CHECK(cudaMalloc(&ptr, nbytes));
      device_buffers[name] = ptr;
      if (!context->setTensorAddress(name, ptr)) {
        std::cerr << "Failed to set tensor address: " << name << std::endl;
        return 1;
      }
    }

    cabot_dnn_controller::tensorrt_utils::copyFloatHostToDevice(
      device_buffers.at(kInputOdomName), h_odom.data(), h_odom.size(),
      engine->getTensorDataType(kInputOdomName), stream);
    cabot_dnn_controller::tensorrt_utils::copyFloatHostToDevice(
      device_buffers.at(kInputPlanName), h_plan.data(), h_plan.size(),
      engine->getTensorDataType(kInputPlanName), stream);
    cabot_dnn_controller::tensorrt_utils::copyFloatHostToDevice(
      device_buffers.at(kInputScanName), h_scan.data(), h_scan.size(),
      engine->getTensorDataType(kInputScanName), stream);
    if (people_encoder_enabled) {
      cabot_dnn_controller::tensorrt_utils::copyFloatHostToDevice(
        device_buffers.at(kInputPeopleName), h_people.data(), h_people.size(),
        engine->getTensorDataType(kInputPeopleName), stream);
    }

    if (!context->enqueueV3(stream)) {
      std::cerr << "TensorRT enqueueV3 failed" << std::endl;
      return 1;
    }

    std::vector<float> h_people_attention;
    std::vector<float> h_robot_people_attention;
    if (people_encoder_enabled) {
      const size_t head_count = static_cast<size_t>(num_attention_heads);
      const size_t person_count = static_cast<size_t>(num_people);
      h_people_attention.resize(head_count * person_count * person_count);
      h_robot_people_attention.resize(head_count * person_count);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        h_people_attention.data(), device_buffers.at(kOutputPeopleAttentionName),
        h_people_attention.size(), engine->getTensorDataType(kOutputPeopleAttentionName), stream);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        h_robot_people_attention.data(), device_buffers.at(kOutputRobotPeopleAttentionName),
        h_robot_people_attention.size(),
        engine->getTensorDataType(kOutputRobotPeopleAttentionName), stream);
    }

    float v_pred = 0.0;
    float w_pred = 0.0;
    if ((action_mode == ActionMode::kReg) || (action_mode == ActionMode::kMdnReg)) {
      std::vector<float> h_cmd(2, 0.0f);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        h_cmd.data(), device_buffers.at(kOutputCmdName), h_cmd.size(),
        engine->getTensorDataType(kOutputCmdName), stream);
      CUDA_CHECK(cudaStreamSynchronize(stream));
      v_pred = h_cmd[0];
      w_pred = h_cmd[1];
    } else if (action_mode == ActionMode::kCls) {
      std::vector<float> v_logits(v_num_bins, 0.0f);
      std::vector<float> w_logits(w_num_bins, 0.0f);

      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        v_logits.data(), device_buffers.at(kOutputVLogitsName), v_logits.size(),
        engine->getTensorDataType(kOutputVLogitsName), stream);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        w_logits.data(), device_buffers.at(kOutputWLogitsName), w_logits.size(),
        engine->getTensorDataType(kOutputWLogitsName), stream);
      CUDA_CHECK(cudaStreamSynchronize(stream));

      std::vector<float> v_logits_first(v_logits.begin(), v_logits.begin() + v_num_bins);
      std::vector<float> w_logits_first(w_logits.begin(), w_logits.begin() + w_num_bins);
      v_pred = cabot_dnn_controller::classify_utils::valueFromArgmaxLogits(v_logits_first, kVMin, kVMax);
      w_pred = cabot_dnn_controller::classify_utils::valueFromArgmaxLogits(w_logits_first, kWMin, kWMax);
    } else {
      std::vector<float> h_cmd(static_cast<size_t>(v_num_bins) * static_cast<size_t>(w_num_bins) * 2, 0.0f);
      std::vector<float> v_logits(v_num_bins, 0.0f);
      std::vector<float> w_logits(w_num_bins, 0.0f);

      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        h_cmd.data(), device_buffers.at(kOutputCmdName), h_cmd.size(),
        engine->getTensorDataType(kOutputCmdName), stream);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        v_logits.data(), device_buffers.at(kOutputVLogitsName), v_logits.size(),
        engine->getTensorDataType(kOutputVLogitsName), stream);
      cabot_dnn_controller::tensorrt_utils::copyDeviceToHostFloat(
        w_logits.data(), device_buffers.at(kOutputWLogitsName), w_logits.size(),
        engine->getTensorDataType(kOutputWLogitsName), stream);
      CUDA_CHECK(cudaStreamSynchronize(stream));

      std::vector<float> v_logits_first(v_logits.begin(), v_logits.begin() + v_num_bins);
      std::vector<float> w_logits_first(w_logits.begin(), w_logits.begin() + w_num_bins);
      auto v_it = std::max_element(v_logits_first.begin(), v_logits_first.end());
      auto w_it = std::max_element(w_logits_first.begin(), w_logits_first.end());
      const size_t v_idx = std::distance(v_logits_first.begin(), v_it);
      const size_t w_idx = std::distance(w_logits_first.begin(), w_it);
      const size_t cmd_idx = v_idx * static_cast<size_t>(w_num_bins) + w_idx;
      v_pred = h_cmd[2 * cmd_idx];
      w_pred = h_cmd[2 * cmd_idx + 1];
    }

    std::cout << "cmd_vel (C++): [" << v_pred << ", " << w_pred << "]" << std::endl;
    std::cout << "cmd_vel (Python): [" << h_out_gt[0] << ", " << h_out_gt[1] << "]" << std::endl;
    std::cout << "cmd_vel (diff): [" << (v_pred - h_out_gt[0]) << ", " << (w_pred - h_out_gt[1]) << "]" << std::endl;

    if (people_encoder_enabled) {
      printFloatVector("people_attention (C++)", h_people_attention);
      printFloatVector("people_attention (Python)", h_people_attention_gt);
      printFloatVector("robot_people_attention (C++)", h_robot_people_attention);
      printFloatVector("robot_people_attention (Python)", h_robot_people_attention_gt);
    }

    for (auto & kv : device_buffers) {
      if (kv.second) {
        CUDA_CHECK(cudaFree(kv.second));
      }
    }
    if (stream) {
      CUDA_CHECK(cudaStreamDestroy(stream));
    }

    return 0;
  } catch (const std::exception & e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
}
