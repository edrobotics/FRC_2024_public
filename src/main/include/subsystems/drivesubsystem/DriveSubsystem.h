// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include <units/constants.h>
#include <frc/RobotBase.h>

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>

#include <frc/smartdashboard/Field2d.h>
#include <wpi/sendable/SendableBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <AHRS.h>
#include <frc/SPI.h>
#include <frc/SerialPort.h>

#include <frc/RobotController.h>
#include "subsystems/drivesubsystem/Drivebasesim.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>

#include <frc/filter/SlewRateLimiter.h>

#include "vision/Vision.h"

#define DISABLE_NAVX2 // This will break simulation when defined
#define DISABLE_VISION

using namespace DriveConstants;

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  /**
   * Drives the drivetrain using ArcadeDrive.
   * 
   * @param fwd forward motor power
   * @param rot rotation
   */
  void arcadeDrive(double fwd, double rot);

  /**
   * Tankdrive using direct motor voltage
   * 
   * @param left Left voltage
   * @param right Right voltage
   */
  void driveVoltage(units::volt_t left, units::volt_t right);

  /**
   * Drives the wheels at the speeds specified in the speeds param
   * 
   * @param speeds A ChassisSpeeds of the desired wheel speeds
   */
  void driveRobotRelative(const frc::ChassisSpeeds& speeds);

  /**
   * Scales the max output of the drivetrain by the value passed in
   * 
   * @param max A value between 0 and 1. 
   */
  void setMaxOutput(double max);

  /**
   * Returns the current pose from the robots odometry
   * 
   * @return A Pose2d of the current pose acording to odometry
   */
  frc::Pose2d getPose();

  /**
   * Resets the odometry to the specified pose
   * 
   * @param pose A Pose2d of the pose to reset to
   */
  void resetOdometry(frc::Pose2d pose);

  /**
   * Returns the current robot speed as a ChassisSpeeds
   * 
   * @return A ChassisSpeeds of the current speeds measured by the encoders
   */
  frc::ChassisSpeeds getSpeeds();

  /**
   * Returns the distance recorded by the Left encoder
   * 
   * @return The distance as units::meter_t
   */
  units::meter_t getLeftEncoderDistance();

  /**
   * Returns the distance recorded by the Right encoder
   * 
   * @return The distance as units::meter_t
   */
  units::meter_t getRightEncoderDistance();

  /**
   * Resets both of the drivetrain encoders to 0
   */
  void resetEncoders();

  /**
   * Sets the yaw of the navX to 0
   */
  void resetNavxHeading();

  /**
   * Returns the heading of the navX
   */
  units::degree_t getNavxHeading();

  /**
   * Returns motor controller faults if any
   * 
   * @return Array of motorcontroller faults in order [LM, LF, RM, RF]
   */
  std::vector<int64_t> getMotorFaults();

  /**
   * Method for initializing the data to send via Network Tables
   */
  void InitSendable(wpi::SendableBuilder& builder) override;

  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  WPI_TalonSRX m_leftMain{kLeftMainMotorID};
  WPI_VictorSPX m_leftFollower{kLeftFollowerMotorID};

  WPI_TalonSRX m_rightMain{kRightMainMotorID};
  WPI_VictorSPX m_rightFollower{kRightFollowerMotorID};

  Faults _faultsLeftMain;
  Faults _faultsLeftFollower;
  Faults _faultsRightMain;
  Faults _faultsRightFollower;

  frc::DifferentialDrive m_robotDrive{[&](double output){m_leftMain.Set(output);},
                                      [&](double output){m_rightMain.Set(output);}};
#ifndef DISABLE_NAVX2
  AHRS m_navX{frc::SerialPort::kUSB1};
#endif
#ifndef DISABLE_VISION
  Vision m_vision;
#endif

  frc::DifferentialDriveKinematics m_kinematics{kTrackWidth};
  frc::DifferentialDrivePoseEstimator m_poseEstimator;
#ifndef DISABLE_NAVX2
  DrivebaseSim s_drivebaseSim{m_leftMain, m_rightMain, m_navX};
#endif
  frc::Field2d _field;

  frc::SlewRateLimiter<units::scalar> rotFilter{3 / 1_s};

  units::meters_per_second_t leftSetSpeed{0_mps};
  units::meters_per_second_t rightSetSpeed{0_mps};
};
