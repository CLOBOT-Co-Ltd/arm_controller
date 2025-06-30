#ifndef ARM_CONTROLLER__DEFINITIONS_HPP_
#define ARM_CONTROLLER__DEFINITIONS_HPP_

#define SIGN(val) ((val >= 0.0) ? 1.0 : -1.0)


#include <string>

const int JOINT_NUMBER = 15; // 상체 조인트 개수 (15개)

const double Pi = 3.141592654;
const double Pi_2 = 1.57079632;
const double Pi_4 = 0.78539816;
const double deg_60 = 1.047198;
const double deg_30 = 0.523599;
const double deg_20 = 0.349066;
const double deg_15 = 0.261799;
const double deg_10 = 0.174533;

const uint8_t GESTURE_INIT_POS = 0;
const uint8_t GESTURE_WAVE_HAND = 1;
const uint8_t GESTURE_FOLLOW_ME = 2;
const uint8_t GESTURE_ITS_ME = 3;
const uint8_t GESTURE_INTRODUCE = 4;
const uint8_t GESTURE_TALKING = 5;

const uint8_t ACTION_GESTURE_SUCCESS = 0;
const uint8_t ACTION_GESTURE_FAILURE = 1;


enum JointIndex
{
  LEFT_SHOULDER_PITCH = 0,
  LEFT_SHOULDER_ROLL = 1,
  LEFT_SHOULDER_YAW = 2,
  LEFT_ELBOW_PITCH = 3,
  LEFT_ELBOW_ROLL = 4,
  LEFT_WRIST_PITCH = 5,
  LEFT_WRIST_YAW = 6,

  RIGHT_SHOULDER_PITCH = 7,
  RIGHT_SHOULDER_ROLL = 8,
  RIGHT_SHOULDER_YAW = 9,
  RIGHT_ELBOW_PITCH = 10,
  RIGHT_ELBOW_ROLL = 11,
  RIGHT_WRIST_PITCH = 12,
  RIGHT_WRIST_YAW = 13,

  WAIST_YAW = 14
};

struct JOINT_INFOS
{
// [0 ~ 14]
//   - LeftShoulderPitch(13), LeftShoulderRoll(14), LeftShoulderYaw(15), LeftElbowPitch(16), LeftElbowRoll(17), LeftWristPitch(18), LeftWristYaw(19)
//   - RightShoulderPitch(20), RightShoulderRoll(21), RightShoulderYaw(22), RightElbowPitch(23), RightElbowRoll(24), RightWristPitch(25), RightWristYaw(26)
//   - WaistYaw(12)

  int joint_number[15];
  double joint_angle_rad[15];
};

struct JOINT_LIMITS
{
// [0 ~ 14]
//   - LeftShoulderPitch(13), LeftShoulderRoll(14), LeftShoulderYaw(15), LeftElbowPitch(16), LeftElbowRoll(17), LeftWristPitch(18), LeftWristYaw(19)
//   - RightShoulderPitch(20), RightShoulderRoll(21), RightShoulderYaw(22), RightElbowPitch(23), RightElbowRoll(24), RightWristPitch(25), RightWristYaw(26)
//   - WaistYaw(12)

  double min_angle_rad[15];
  double max_angle_rad[15];
}


#endif // ARM_CONTROLLER__DEFINITIONS_HPP_
