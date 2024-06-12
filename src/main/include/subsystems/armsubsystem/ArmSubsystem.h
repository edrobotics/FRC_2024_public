// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <frc2/command/CommandPtr.h>
#include <frc2/command/PIDsubsystem.h>

#include <units/constants.h>

#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/controller/PIDController.h>

#include <wpi/sendable/SendableBuilder.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/DigitalInput.h>

#include <frc/DataLogManager.h>
#include <string>

#include <frc/Timer.h>

using namespace ArmConstants;

class ArmSubsystem : public frc2::PIDSubsystem {
 public:
  ArmSubsystem();

  /**
   * Move the arm to the scoring/up position
  */
  void toScoringPosition();
  
  /**
   * Move the arm to the intake/down position
  */
  void toIntakePosition();

  /**
   * Move the arm down
  */
  void down();

  /**
   * Move the arm up
  */
  void up();

  /*
   * Sets the arm motorSpeed to 0
  */
  void stop();

  /*
   * Sets the current position as the encoders zero pose
  */
  void zeroEncoder();

  /**
   * Gets the current angle of the arm
  */
  units::radian_t getArmAngle();

  /**
   * Method that keeps the note state updated based on limit switch data.
  */
  void noteStateUpdater();

  /**
   * Sets the state of the note in the arm
   * 
   * @param state a bool: true if the arm is holding a note, false if not
  */
  void setNoteState(bool state);

  /**
   * Gets whether the arm is currently at the intake position
   *
   * @return true if arm is at the intake position, false if not or if moving.
   */
  bool armIsDown();

  /**
   * Gets whether the arm is currently holding a note
   * 
   * @return true if the arm is holding a note, false if not
  */
  bool noteInIntake();

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
   * Returns the encoder angle as a double for the PID
   * 
   * @return double representing encoder angle
  */
  virtual double GetMeasurement();

  /**
   * Consumes the output of the PID controller and uses it to move the arm
   * 
   * @param output the output of the PIDController 
   * @param setpoint the setpoint of the PIDController (for feedforward)
  */
  virtual void UseOutput(double output, double setpoint);

  /**
   * Method for initializing the data to send via Network Tables
   */
  void InitSendable(wpi::SendableBuilder& builder) override;

  bool disablearmup{false};
  bool disablearmdown{false};

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  /**
   * Utility method to run during periodic for checking and enforcing arm safety
  */
  void armSafety();

  bool notestate{false};

  rev::CANSparkMax m_armMotor{2, rev::CANSparkMax::MotorType::kBrushless};

  frc::DutyCycleEncoder m_encoder{0};

  frc::DigitalInput m_noteSwitch{1};

  // A variable for logging the arm offset before the competition.
  double armoffset{0};

  frc::Timer m_timer;
};
