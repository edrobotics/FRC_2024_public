// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#define M_PI 3.14159265358979323846

#pragma once

#include <units/constants.h>
#include <units/angle.h>
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
inline constexpr int kOperatorController = 1;

}  // namespace OperatorConstants

namespace DriveConstants {

constexpr int kLeftMainMotorID = 1;
constexpr int kLeftFollowerMotorID = 1;
constexpr int kRightMainMotorID = 2;
constexpr int kRightFollowerMotorID = 2;

constexpr units::length::centimeter_t kTrackWidth = 57.4_cm;
constexpr int kEncoderUnitsPerRotation = 4046;
constexpr double kEncoderWheelGearRatio = 1;
constexpr units::inch_t kWheelDiameter = 6_in;
constexpr double kMotorGearRatio = 10.71; // Needs to be updated with actual value

// Rate is measured in encoder units per 100ms:
constexpr units::second_t kVelocityPollingInterval = 0.1_s;
// set to zero to skip waiting for confirmation, set to nonzero to wait and report to DS if action fails.
constexpr int kTimeoutMs = 30;
constexpr int kSlotIdx = 0;

constexpr double kF = 0.299941;
constexpr double kP_left = 0.0;
constexpr double kP_right = 0.0;
constexpr double kI_left = 0.0;
constexpr double kI_right = 0.0;
constexpr double kD_left = 0.0;
constexpr double kD_right = 0.0;

// Conversion functions for ctre sensor units
constexpr units::meter_t encoderUnitsToDistance(double units) {
  double rotations = (units / kEncoderUnitsPerRotation) / kEncoderWheelGearRatio;
  return rotations * (M_PI * kWheelDiameter);
}
constexpr units::meters_per_second_t encoderUintsToVelocity(double units) {
  double rotations = (units / kEncoderUnitsPerRotation) / kEncoderWheelGearRatio;
  units::meter_t distance = rotations * (M_PI * kWheelDiameter);
  return distance / kVelocityPollingInterval;
}
constexpr double distanceToEncoderUnits(units::meter_t distance) {
  double rotations = distance / (M_PI * kWheelDiameter);
  return (rotations * kEncoderWheelGearRatio) * kEncoderUnitsPerRotation;
}
constexpr double velocityToEncoderUnits(units::meters_per_second_t velocity) {
  units::meter_t distance = velocity * kVelocityPollingInterval;
  double rotations = distance / (M_PI * kWheelDiameter);
  return (rotations * kEncoderWheelGearRatio) * kEncoderUnitsPerRotation;
}
} // Drive Subsystem Constants

namespace VisionConstants {

  // How far "forward" the cameras are
  const units::length::meter_t kCamera11X = -0.25_m;
  const units::length::meter_t kCamera12X = 0.28_m;

  // How far "left/right" the cameras are
  const units::length::meter_t kCamera11Y = 0.0_m;
  const units::length::meter_t kCamera12Y = 0.0_m;

  // How far "up" the cameras are
  const units::length::meter_t kCamera11Z = 0.55_m;
  const units::length::meter_t kCamera12Z = 0.15_m;
  
  // The roll of the cameras
  const units::angle::radian_t kCamera11Roll = 0.0_rad;
  const units::angle::radian_t kCamera12Roll = 0.0_rad;

  // The pitch of the cameras
  const units::angle::radian_t kCamera11Pitch = -20_deg;
  const units::angle::radian_t kCamera12Pitch = -20_deg;

  // The yaw of the cameras
  const units::angle::radian_t kCamera11Yaw = 0.0_rad;
  const units::angle::radian_t kCamera12Yaw = (units::angle::radian_t)M_PI;
  
} // namespace VisionConstants

namespace ArmConstants {
  constexpr double kP = 0.5;
  constexpr double kI = 0;
  constexpr double kD = 0;

  constexpr double kConstArmSpeed = 0.42;

  constexpr double kPoseOffset = 3.69 / (M_PI * 2);
  constexpr units::radian_t kIntakePose = 0_rad;
  constexpr units::radian_t kScoringPose = units::radian_t(M_PI / 2);
  constexpr units::radian_t kExpansionLimit = 2.6_rad; 
  constexpr units::radian_t kSafetyMargin = 0.15_rad;
} // Arm subsystem constants