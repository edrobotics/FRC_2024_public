// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveForwardCommand.h"

DriveForwardCommand::DriveForwardCommand(DriveSubsystem* subsystem)
  : m_drive(subsystem) 
{
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(m_drive);
}

// Called when the command is initially scheduled.
void DriveForwardCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void DriveForwardCommand::Execute() {
  distance = (m_drive->getLeftEncoderDistance() + m_drive->getRightEncoderDistance()) / 2.0;

  if (distance < 3.0_m) {
    m_drive->driveVoltage(3.0_V, 2.8_V); // These voltages are based on rudimentary testing
  }
  else if (distance >= 3.0_m) {
    m_drive->driveVoltage(0.0_V, 0.0_V);
  }
}

// Called once the command ends or is interrupted.
void DriveForwardCommand::End(bool interrupted) {
  m_drive->driveVoltage(0.0_V, 0.0_V);
}

// Returns true when the command should end.
bool DriveForwardCommand::IsFinished() {
  return false;
}
