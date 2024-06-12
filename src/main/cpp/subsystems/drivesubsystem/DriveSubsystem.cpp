// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/drivesubsystem/DriveSubsystem.h"

DriveSubsystem::DriveSubsystem() 
  : m_poseEstimator{
    m_kinematics,
    getNavxHeading(),
    getLeftEncoderDistance(),
    getRightEncoderDistance(),
    frc::Pose2d{0_m, 0_m, 0_rad},
    {0.02, 0.02, 0.01}, // These are curently the standard values,
    {0.1, 0.1, 0.1}     // should probably get arround to tuning this sometime.
  }
{
  m_leftMain.ConfigFactoryDefault();
  m_leftFollower.ConfigFactoryDefault();
  m_rightMain.ConfigFactoryDefault();
  m_rightFollower.ConfigFactoryDefault();

  m_leftMain.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);
  m_rightMain.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 10);

  m_leftFollower.Follow(m_leftMain);
  m_rightFollower.Follow(m_rightMain);

  // Invert left (might need to be changed on actual hardware).
  m_leftMain.SetInverted(false);
  m_rightMain.SetInverted(true);

  // This will be decided based on the gearbox design.
  m_leftFollower.SetInverted(InvertType::FollowMaster);
  m_rightFollower.SetInverted(InvertType::FollowMaster);
  // Might need to be adjusted depending on encoder mounting
  m_leftMain.SetSensorPhase(false);
  m_rightMain.SetSensorPhase(false);

  m_leftMain.ConfigNominalOutputForward(0, kTimeoutMs);
  m_leftMain.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_leftMain.ConfigPeakOutputForward(1, kTimeoutMs);
  m_leftMain.ConfigPeakOutputReverse(-1, kTimeoutMs);

  m_leftMain.Config_kF(kSlotIdx, kF, kTimeoutMs);
	m_leftMain.Config_kP(kSlotIdx, kP_left, kTimeoutMs);
	m_leftMain.Config_kI(kSlotIdx, kI_left, kTimeoutMs);
	m_leftMain.Config_kD(kSlotIdx, kD_left, kTimeoutMs);

  m_rightMain.ConfigNominalOutputForward(0, kTimeoutMs);
  m_rightMain.ConfigNominalOutputReverse(0, kTimeoutMs);
  m_rightMain.ConfigPeakOutputForward(1, kTimeoutMs);
  m_rightMain.ConfigPeakOutputReverse(-1 ,kTimeoutMs);

  m_rightMain.Config_kF(kSlotIdx, kF, kTimeoutMs);
	m_rightMain.Config_kP(kSlotIdx, kP_right, kTimeoutMs);
	m_rightMain.Config_kI(kSlotIdx, kI_right, kTimeoutMs);
	m_rightMain.Config_kD(kSlotIdx, kD_right, kTimeoutMs);

  // Trying this it might not be the right choice
  m_rightMain.SetNeutralMode(motorcontrol::NeutralMode::Coast);
  m_leftMain.SetNeutralMode(motorcontrol::NeutralMode::Coast);

  resetOdometry(frc::Pose2d(1.0_m, 1.0_m, units::degree_t(0.0)));

  pathplanner::AutoBuilder::configureRamsete(
        [this](){ return getPose(); }, // Robot pose supplier
        [this](frc::Pose2d pose){ resetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
        [this](){ return getSpeeds(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        [this](frc::ChassisSpeeds speeds){ driveRobotRelative(speeds); }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        units::unit_t<frc::RamseteController::b_unit>(2.0),
        units::unit_t<frc::RamseteController::zeta_unit>(0.7),
        pathplanner::ReplanningConfig(), // Default path replanning config. See the API for the options here
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            auto alliance = frc::DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == frc::DriverStation::Alliance::kRed;
            }
            return false;
        },
        this // Reference to this subsystem to set requirements
    );

  frc::SmartDashboard::PutData("Field", &_field);

  pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this] (frc::Pose2d pose) {
    this->_field.GetObject("target pose")->SetPose(pose);
  });
  // Set up custom logging to add the current path to a field 2d widget
  pathplanner::PathPlannerLogging::setLogActivePathCallback([this] (std::vector<frc::Pose2d> poses) {
      this->_field.GetObject("path")->SetPoses(poses);
  });
}

// This method will be called once per scheduler run
void DriveSubsystem::Periodic() {
  m_poseEstimator.Update(getNavxHeading(),
                    getLeftEncoderDistance(),
                    getRightEncoderDistance());

#ifndef DISABLE_VISION
  if (!frc::RobotBase::IsSimulation) {
    m_vision.update(m_poseEstimator);
  }
#endif

  _field.SetRobotPose(getPose());
  
  m_leftMain.GetFaults(_faultsLeftMain);
  m_leftFollower.GetFaults(_faultsLeftFollower);
  m_rightMain.GetFaults(_faultsRightMain);
  m_rightFollower.GetFaults(_faultsRightFollower);
}

void DriveSubsystem::SimulationPeriodic() {
#ifndef DISABLE_NAVX2
  s_drivebaseSim.runDrivetrainSim();
#endif
}

void DriveSubsystem::arcadeDrive(double fwd, double rot) {
  if (fwd < -0.1) {
    rot = -rot;
  }
  m_robotDrive.ArcadeDrive(fwd, rotFilter.Calculate(rot));
}

void DriveSubsystem::driveVoltage(units::volt_t left, units::volt_t right) {
  m_leftMain.SetVoltage(left);
  m_rightMain.SetVoltage(right);
  m_robotDrive.Feed();
}

void DriveSubsystem::driveRobotRelative(const frc::ChassisSpeeds& speeds) {
  frc::DifferentialDriveWheelSpeeds wheelSpeeds{m_kinematics.ToWheelSpeeds(speeds)};

  m_leftMain.Set(ControlMode::Velocity, velocityToEncoderUnits(wheelSpeeds.left));
  m_rightMain.Set(ControlMode::Velocity, velocityToEncoderUnits(wheelSpeeds.right));

  m_robotDrive.Feed();

  leftSetSpeed = wheelSpeeds.left;
  rightSetSpeed = wheelSpeeds.right;
}

void DriveSubsystem::setMaxOutput(double max) {
  m_robotDrive.SetMaxOutput(max);
}

frc::Pose2d DriveSubsystem::getPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::resetOdometry(frc::Pose2d pose) {
  resetEncoders();
  m_poseEstimator.ResetPosition(getNavxHeading(),
                           getLeftEncoderDistance(),
                           getRightEncoderDistance(),
                           pose);
}

frc::ChassisSpeeds DriveSubsystem::getSpeeds() {
  return m_kinematics.ToChassisSpeeds(
    {encoderUintsToVelocity(m_leftMain.GetSelectedSensorVelocity(kSlotIdx)),
     encoderUintsToVelocity(m_rightMain.GetSelectedSensorVelocity(kSlotIdx))});
}

units::meter_t DriveSubsystem::getLeftEncoderDistance() {
  return encoderUnitsToDistance(m_leftMain.GetSelectedSensorPosition(kSlotIdx));
}

units::meter_t DriveSubsystem::getRightEncoderDistance() {
  return encoderUnitsToDistance(m_rightMain.GetSelectedSensorPosition(kSlotIdx));
}

void DriveSubsystem::resetEncoders() {
  m_leftMain.SetSelectedSensorPosition(0, 0);
  m_rightMain.SetSelectedSensorPosition(0, 0);
}

void DriveSubsystem::resetNavxHeading() {
#ifndef DISABLE_NAVX2
  m_navX.ZeroYaw();
#endif
}

units::degree_t DriveSubsystem::getNavxHeading() {
  // This probably needs to be negated as I don't think it's CCW positive
#ifndef DISABLE_NAVX2
  return units::degree_t(-m_navX.GetAngle());
#else
  return units::degree_t(0.0);
#endif
}

std::vector<int64_t> DriveSubsystem::getMotorFaults() {
  std::vector<int64_t> faults{_faultsLeftMain.ToBitfield(), 
                              _faultsLeftFollower.ToBitfield(),
                              _faultsRightMain.ToBitfield(),
                              _faultsRightFollower.ToBitfield()};
  return faults;
}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  SubsystemBase::InitSendable(builder);
  builder.SetSmartDashboardType("Drive Subsystem");

  builder.AddDoubleProperty("Left Encoder Distance", 
    [this] {return getLeftEncoderDistance().value();}, nullptr);
  builder.AddDoubleProperty("Right Encoder Distance", 
    [this] {return getRightEncoderDistance().value();}, nullptr);

  builder.AddDoubleProperty("Left Encoder Velocity", 
    [this] {
      return encoderUintsToVelocity(m_leftMain.GetSelectedSensorVelocity()).value();
      }, nullptr);
  builder.AddDoubleProperty("Right Encoder Velocity", 
    [this] {
      return encoderUintsToVelocity(m_rightMain.GetSelectedSensorVelocity()).value();
      }, nullptr);

  builder.AddDoubleProperty("Left Requested Speed", 
    [this] {return leftSetSpeed.value();}, nullptr);
  builder.AddDoubleProperty("Right Requested Speed", 
    [this] {return rightSetSpeed.value();}, nullptr);

  builder.AddDoubleProperty("Left Voltage", 
    [this] {return m_leftMain.GetMotorOutputVoltage();}, nullptr);
  builder.AddDoubleProperty("Right Voltage", 
    [this] {return m_rightMain.GetMotorOutputVoltage();}, nullptr);

  builder.AddDoubleProperty("IMU angle", 
    [this] {return getNavxHeading().value();}, nullptr);

  builder.AddIntegerArrayProperty("Motorcontroller Faults", 
    [this] {return getMotorFaults();}, nullptr);
#ifndef DISABLE_VISION
  builder.AddIntegerArrayProperty("Current target IDs", 
    [this] {return m_vision.getCurrentTargetIDs();}, nullptr);
#endif
}
