// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/Commands.h>
#include <frc2/command/RunCommand.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "commands/LowerDriveSpeed.h"
#include "commands/DriveForwardCommand.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Starts recording to data log,
  // this will record all data sent over networktables
  frc::DataLogManager::Start();

  // Configure the button bindings
  ConfigureBindings();

  // Set arcadedrive as the default controll method.
  m_drive.SetDefaultCommand(frc2::RunCommand(
    [this] {
      m_drive.arcadeDrive(-m_driverController.GetLeftY(),
                          -m_driverController.GetRightX());
    }, 
    {&m_drive}
  ));


  frc::SmartDashboard::PutData("Drive Subsystem", &m_drive);
  frc::SmartDashboard::PutData("Arm Subsystem", &m_arm);
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  
  /**
   * ##############################################################################################
   *                                     Driver Controls
   * ##############################################################################################
  */
  m_driverController.RightBumper().WhileTrue(LowerDriveSpeed(&m_drive).ToPtr());

  //Spin the intake if leftBumper on the controller is pressed and the arm is down
  m_driverController.LeftBumper().Debounce(50_ms, frc::Debouncer::DebounceType::kBoth).OnTrue(
    frc2::cmd::RunOnce( 
      [this] { m_intake.spin(); }, //Spin the intake
      {&m_intake}
    ).OnlyWhile([this]{return m_arm.armIsDown();}) //Interrupt if the arm is not down
  )
  .OnFalse(
    frc2::cmd::RunOnce(
      [this] { m_intake.stopSpin(); },
      {&m_intake}
    )
  );

  m_driverController.Y().Debounce(50_ms, frc::Debouncer::DebounceType::kBoth).OnTrue(
    frc2::cmd::RunOnce(
      [this] { m_arm.up(); }, // Lift the arm up
      {&m_arm}
    ).OnlyWhile([this] { return !m_arm.disablearmup; })
  )
  .OnFalse(
    frc2::cmd::RunOnce(
      [this] { m_arm.stop(); },
      {&m_arm}
    )
  );
  m_driverController.X().Debounce(50_ms, frc::Debouncer::DebounceType::kBoth).OnTrue(
    frc2::cmd::RunOnce(
      [this] { m_arm.down(); },
      {&m_arm}
    ).OnlyWhile([this] { return !m_arm.disablearmdown; })
  )
  .OnFalse(
    frc2::cmd::RunOnce(
      [this] { m_arm.stop(); },
      {&m_arm}
    )
  );

  //Also interrupt if leftTrigger on the controller is pressed
  m_driverController.LeftTrigger().OnTrue(frc2::cmd::Idle({&m_intake})); 
  
  /**
   * ##############################################################################################
   *                                   Operator Controls
   * ##############################################################################################
  */
  
  // These methods enable and disable the PID
  m_operatorController.A().OnTrue(frc2::cmd::RunOnce(
      [this] { m_arm.Disable(); }
    )
  );
  m_operatorController.B().OnTrue(frc2::cmd::RunOnce(
      [this] { m_arm.Enable(); }
    )
  );

  // Set the arms target state
  m_operatorController.RightBumper().OnTrue(frc2::cmd::RunOnce(
      [this] { m_arm.toScoringPosition(); },
      {&m_arm}
    )
  );
  m_operatorController.RightTrigger().OnTrue(frc2::cmd::RunOnce(
      [this] { m_arm.toIntakePosition(); },
      {&m_arm}
    )
  );

  // Spin the intake
  m_operatorController.LeftBumper().OnTrue(
    frc2::cmd::RunOnce( 
      [this] { m_intake.spin(); }, //Spin the intake
      {&m_intake}
    ).OnlyWhile([this]{return m_arm.armIsDown();}) //Interrupt if the arm is not down
  )
  .OnFalse(
    frc2::cmd::RunOnce(
      [this] { m_intake.stopSpin(); },
      {&m_intake}
    )
  );

  // Move the arm manualy 
  m_operatorController.Y().Debounce(50_ms, frc::Debouncer::DebounceType::kBoth).OnTrue(
    frc2::cmd::RunOnce(
      [this] { m_arm.up(); }, // Lift the arm up
      {&m_arm}
    ).OnlyWhile([this] { return !m_arm.disablearmup; })
  )
  .OnFalse(
    frc2::cmd::RunOnce(
      [this] { m_arm.stop(); },
      {&m_arm}
    )
  );
  m_operatorController.X().Debounce(50_ms, frc::Debouncer::DebounceType::kBoth).OnTrue(
    frc2::cmd::RunOnce(
      [this] { m_arm.down(); },
      {&m_arm}
    ).OnlyWhile([this] { return !m_arm.disablearmdown; })
  )
  .OnFalse(
    frc2::cmd::RunOnce(
      [this] { m_arm.stop(); },
      {&m_arm}
    )
  );
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return DriveForwardCommand(&m_drive).ToPtr(); // Remember to uncoment auton in robot.cpp if we are running it.
}
