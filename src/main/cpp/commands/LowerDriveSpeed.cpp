// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/LowerDriveSpeed.h"

LowerDriveSpeed::LowerDriveSpeed(DriveSubsystem* subsystem) 
  : m_drive(subsystem)
{
  // Use addRequirements() here to declare subsystem dependencies.
  // AddRequirements(subsystem);
}

// Called when the command is initially scheduled.
void LowerDriveSpeed::Initialize() {
  m_drive->setMaxOutput(0.50);
}

// Called once the command ends or is interrupted.
void LowerDriveSpeed::End(bool interrupted) {
  m_drive->setMaxOutput(1);
}
