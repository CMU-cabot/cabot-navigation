#ifndef CABOT_DNN_CONTROLLER__DNN_CONTROLLER_CONSTANTS_HPP_
#define CABOT_DNN_CONTROLLER__DNN_CONTROLLER_CONSTANTS_HPP_

#include <cstddef>

namespace cabot_dnn_controller
{
namespace dnn_controller_constants
{

enum class ActionMode {
  kReg,
  kCls,
  kClsReg,
  kMdnReg,
};
  
// TODO: set from navigation parameter
static constexpr std::size_t kScanLength = 1200;
static constexpr float kScanRangeMax = 50.0;
static constexpr int kPeopleDim = 3;  // x, y, presence
static constexpr int kPeopleDimWithVelocity = 5;  // x, y, relative vx, relative vy, presence

static constexpr float kVMin = 0.0;
static constexpr float kVMax = 1.0;
static constexpr float kWMin = -1.0;
static constexpr float kWMax = 1.0;

static constexpr const char * kInputOdomName = "odom";
static constexpr const char * kInputPlanName = "plan";
static constexpr const char * kInputScanName = "scan";
static constexpr const char * kInputPeopleName = "people";
static constexpr const char * kOutputCmdName = "cmd_vel";
static constexpr const char * kOutputVLogitsName = "v_logits";
static constexpr const char * kOutputWLogitsName = "w_logits";
static constexpr const char * kOutputPeopleAttentionName = "people_attention";
static constexpr const char * kOutputRobotPeopleAttentionName = "robot_people_attention";

}  // namespace dnn_controller_constants
}  // namespace cabot_dnn_controller

#endif  // CABOT_DNN_CONTROLLER__DNN_CONTROLLER_CONSTANTS_HPP_
