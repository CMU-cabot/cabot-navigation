#ifndef CABOT_DNN_CONTROLLER__CLASSIFY_UTILS_HPP_
#define CABOT_DNN_CONTROLLER__CLASSIFY_UTILS_HPP_

#include <algorithm>
#include <vector>

namespace cabot_dnn_controller
{
namespace classify_utils
{

float valueFromArgmaxLogits(const std::vector<float>& logits, float x_min, float x_max) {
    if (logits.empty()) return 0.0f;

    const size_t n = logits.size();
    if (n == 1) return x_min;

    auto it = std::max_element(logits.begin(), logits.end());
    const size_t max_idx = std::distance(logits.begin(), it);

    return x_min + (x_max - x_min) * (static_cast<float>(max_idx) / static_cast<float>(n - 1));
}

}  // namespace classify_utils
}  // namespace cabot_dnn_controller

#endif  // CABOT_DNN_CONTROLLER__CLASSIFY_UTILS_HPP_
