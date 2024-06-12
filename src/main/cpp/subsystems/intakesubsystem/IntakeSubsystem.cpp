// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/intakesubsystem/IntakeSubsystem.h"

IntakeSubsystem::IntakeSubsystem() {
  // Implementation of subsystem constructor goes here.
}

void IntakeSubsystem::spin() {
  m_intakeMotor.Set(0.65);
}

void IntakeSubsystem::stopSpin() {
  m_intakeMotor.Set(0.0);
}


void IntakeSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
}

void IntakeSubsystem::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}