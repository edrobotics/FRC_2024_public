// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/armsubsystem/ArmSubsystem.h"

#define M_PI 3.14159265358979323846

ArmSubsystem::ArmSubsystem() 
  : PIDSubsystem{frc::PIDController{kP, kI, kD}}
{
  // Implementation of subsystem constructor goes here.
  //call toIntakePosition()?
  m_armMotor.SetInverted(true);
  m_armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
  //Set the distance to radians
  m_encoder.SetDistancePerRotation(2*M_PI);
  //Set the PID tolerance
  m_controller.SetTolerance(0.2);
  // Set the initial pid setpoint to intake pose
  toIntakePosition(); 

  armoffset = m_encoder.GetAbsolutePosition();

  m_timer.Start();

  frc::DataLogManager::Log(std::to_string(armoffset));
}

void ArmSubsystem::toScoringPosition()
{
  SetSetpoint(kScoringPose.value());
}

void ArmSubsystem::toIntakePosition()
{
  SetSetpoint(kIntakePose.value());
}

void ArmSubsystem::down() {
  if (!disablearmdown)
    m_armMotor.Set(-kConstArmSpeed);
}

void ArmSubsystem::up() {
  if (!disablearmup)
    m_armMotor.Set(kConstArmSpeed);
}

void ArmSubsystem::stop() {
  m_armMotor.Set(0.0);
}

void ArmSubsystem::zeroEncoder() {
  m_encoder.SetPositionOffset(m_encoder.GetDistance());
}

units::radian_t ArmSubsystem::getArmAngle() {
  return units::radian_t(m_encoder.GetDistance());
}

void ArmSubsystem::noteStateUpdater() {

}

void ArmSubsystem::setNoteState(bool state) {
  notestate = state;
}

bool ArmSubsystem::armIsDown() {
  // Check if arm is down
  if (getArmAngle() < 0.1_rad && getArmAngle() > -0.1_rad) {
    return true;
  }
  else {
    return false;
  }
}

bool ArmSubsystem::noteInIntake() {
  return notestate;
}

void ArmSubsystem::Periodic() {
  frc2::PIDSubsystem::Periodic();
  // Implementation of subsystem periodic method goes here.

  armSafety();

  // Check limit m_noteSwitch

  if (m_timer.Get() < 500_ms) {
    armoffset = m_encoder.GetAbsolutePosition();
  }
  
  if (!(armoffset == m_encoder.GetPositionOffset())) {
    m_encoder.SetPositionOffset(armoffset);
  }
}

void ArmSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

double ArmSubsystem::GetMeasurement() {
  return getArmAngle().value();
}

void ArmSubsystem::UseOutput(double output, double setpoint) {
  if (output && !(disablearmup || disablearmdown) )
  m_armMotor.SetVoltage(units::volt_t(output));

  frc::SmartDashboard::PutNumber("Arm voltage", output);
}

void ArmSubsystem::armSafety() {
  units::radian_t armpose = getArmAngle();
  //double motoroutput = m_armMotor.Get();

  // Checks if the arm has gone outside its boundaries
  if (((kIntakePose - kSafetyMargin) > armpose)) { // intake
    disablearmdown = true;
    Disable(); // disable the PID
  }
  else if (((kExpansionLimit - kSafetyMargin) < armpose)) //arm
  {
    disablearmup = true;
    Disable(); // disable the pid

    if (m_armMotor.GetIdleMode() == rev::CANSparkMax::IdleMode::kCoast) {
      m_armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    }
  }
  // check for if the arm is brakeing when not at risk of overextending
  else if ((kExpansionLimit - 2 * kSafetyMargin) > armpose) { 
    disablearmup = false;
    if (m_armMotor.GetIdleMode() == rev::CANSparkMax::IdleMode::kBrake) {
      m_armMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    }
  }
  frc::SmartDashboard::PutBoolean("disablearmup", disablearmup);
  frc::SmartDashboard::PutBoolean("disablearmdown", disablearmdown);
}

void ArmSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  SubsystemBase::InitSendable(builder);
  builder.SetSmartDashboardType("Arm Subsystem");
  
  builder.AddDoubleProperty("Encoder pose", 
    [this] { return getArmAngle().value(); }, 
    [this] (double offset) { m_encoder.SetPositionOffset(offset); }
  );

  builder.AddBooleanProperty("Is note in arm", 
    [this] { return noteInIntake(); }, 
    [this] (bool state) { setNoteState(state); }
  );

  builder.AddDoubleProperty("Arm controller error",
    [this] { return GetController().GetPositionError();}, 
    nullptr
  );

  builder.AddDoubleProperty("Arm setpoint",
    [this] { return GetSetpoint(); },
    [this] (double setpoint) { SetSetpoint(setpoint);}
  );

  builder.AddBooleanProperty("Is arm down",
    [this] { return armIsDown(); },
    nullptr
  );

  builder.AddBooleanProperty("Is PID enabled",
    [this] { return IsEnabled(); },
    nullptr
  );

  builder.AddDoubleProperty("Arm motor voltage",
    [this] { return m_armMotor.GetAppliedOutput(); },
    nullptr
  );
}

//Functions:
//  Score - combined command
//    ArmToBasePos
//    ArmToScorePos
//  IsDown - for intake
