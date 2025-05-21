#ifndef ARM_CONTROLLER__DEFINITIONS_HPP_
#define ARM_CONTROLLER__DEFINITIONS_HPP_

#include <string>

const int JOINT_NUMBER = 15; // 상부 조인트 모터 개수 (15개)


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


#endif // ARM_CONTROLLER__DEFINITIONS_HPP_
