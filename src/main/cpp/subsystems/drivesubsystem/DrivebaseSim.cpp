// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/drivesubsystem/DrivebaseSim.h"

DrivebaseSim::DrivebaseSim(WPI_TalonSRX& leftMain, WPI_TalonSRX& rightMain, AHRS& navX) 
  : _leftMain{leftMain}, _rightMain{rightMain}, _leftMainSim{_leftMain.GetSimCollection()},
  _rightMainSim{_rightMain.GetSimCollection()}, _navX{navX}
{
  _navXsim = HALSIM_GetSimDeviceHandle("navX-Sensor[4]");
  simGyroYaw = HALSIM_GetSimValueHandle(_navXsim, "Yaw");
}

void DrivebaseSim::runDrivetrainSim() {
  /* Pass the robot battery voltage to the simulated Talon SRXs */
	_leftMainSim.SetBusVoltage(frc::RobotController::GetInputVoltage());
	_rightMainSim.SetBusVoltage(frc::RobotController::GetInputVoltage());

	/*
	 * CTRE simulation is low-level, so SimCollection inputs
	 * and outputs are not affected by SetInverted(). Only
	 * the regular user-level API calls are affected.
	 *
	 * WPILib expects +V to be forward.
	 * Positive motor output lead voltage is ccw. We observe
	 * on our physical robot that this is reverse for the
	 * right motor, so negate it.
	 *
	 * We are hard-coding the negation of the values instead of
	 * using GetInverted() so we can catch a possible bug in the
	 * robot code where the wrong value is passed to SetInverted().
	 */
	_driveSim.SetInputs(_leftMainSim.GetMotorOutputLeadVoltage() * 1_V,
						-_rightMainSim.GetMotorOutputLeadVoltage() * 1_V);

	/*
	 * Advance the model by 20 ms. Note that if you are running this
	 * subsystem in a separate thread or have changed the nominal
	 * timestep of TimedRobot, this value needs to match it.
	 */
	_driveSim.Update(20_ms);

	/*
	 * Update all of our sensors.
	 *
	 * Since WPILib's simulation class is assuming +V is forward,
	 * but -V is forward for the right motor, we need to negate the
	 * position reported by the simulation class. Basically, we
	 * negated the input, so we need to negate the output.
	 *
	 * We also observe on our physical robot that a positive voltage
	 * across the output leads results in a negative sensor velocity
	 * for both the left and right motors, so we need to negate the
	 * output once more.
	 * Left output: +1 * -1 = -1
	 * Right output: -1 * -1 = +1
	 */
	_leftMainSim.SetQuadratureRawPosition(
					distanceToEncoderUnits(
						-_driveSim.GetLeftPosition()
					));
	_leftMainSim.SetQuadratureVelocity(
					velocityToEncoderUnits(
						-_driveSim.GetLeftVelocity()
					));
	_rightMainSim.SetQuadratureRawPosition(
					distanceToEncoderUnits(
						_driveSim.GetRightPosition()
					));
	_rightMainSim.SetQuadratureVelocity(
					velocityToEncoderUnits(
						_driveSim.GetRightVelocity()
					));

  // Set IMU heading, **this is not a full IMU simulation**.
  setGyroYaw(_driveSim.GetHeading().Radians());
	frc::SmartDashboard::PutNumber("Sim heading", 
		_driveSim.GetHeading().Degrees().value());
}

void DrivebaseSim::setGyroYaw(units::radian_t newYaw) {
	simGyroYaw.Set((-units::convert<units::radian, units::degree>(newYaw) + internalOffset).to<double>());
}

void DrivebaseSim::setOffset(units::radian_t offset) {
  internalOffset = offset;
}
