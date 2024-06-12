// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include <units/constants.h>

#include <ctre/phoenix.h>

#include <AHRS.h>
#include <frc/SPI.h>
#include <hal/SimDevice.h>
#include <hal/simulation/SimDeviceData.h>

#include <frc/RobotController.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>

#include <frc/smartdashboard/SmartDashboard.h>

using namespace DriveConstants;

class DrivebaseSim {
 public:
  /**
	 * Creates a new drivebase simualtor using Talon SRX motor controllers.
	 *
	 * @param leftMain the left main Talon SRX
	 * @param rightMain the right main Talon SRX
   * @param navX a navX gyro, (i'm not certain if it's needed)
   */
  DrivebaseSim(WPI_TalonSRX& leftMain, WPI_TalonSRX& rightMain, AHRS& navX);

   /**
   * A method that collects all of the functions that are needed to
   * run the drivetrain simulation.
   * 
   * The code for this is almost entirely stolen from the phoenix 
   * simulation example:
   * https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/C%2B%2B%20General/DifferentialDrive/src/main/cpp/DrivebaseSimSRX.cpp
   */
  void runDrivetrainSim();

 private:

  WPI_TalonSRX& _leftMain;
	WPI_TalonSRX& _rightMain;

	TalonSRXSimCollection& _leftMainSim;
	TalonSRXSimCollection& _rightMainSim;

  HAL_SimDeviceHandle _navXsim;
  hal::SimDouble simGyroYaw;
  units::radian_t internalOffset{0};

  AHRS& _navX;

  void setGyroYaw(units::radian_t newYaw);
  void setOffset(units::radian_t offset);

  //These numbers are an example AndyMark Drivetrain with some additional weight.  This is a fairly light robot.
	//Note you can utilize results from robot characterization instead of theoretical numbers.
	//https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
	//Simulation model of the drivetrain
	frc::sim::DifferentialDrivetrainSim _driveSim{
		frc::DCMotor::CIM(2),        //2 CIM motors on each side
		kMotorGearRatio,                  //Standard AndyMark Gearing reduction.
		2.1_kg_sq_m,                 //MOI of 2.1 kg m^2 (from CAD model).
		26.5_kg,                     //Mass of the robot is 26.5 kg.
		kWheelDiameter / 2,          //Robot uses 3" radius (6" diameter) wheels.
		kTrackWidth,                     //Distance between wheels is _ meters.

		// The standard deviations for measurement noise:
		// x and y:          0.001 m
		// heading:          0.001 rad
		// l and r velocity: 0.1   m/s
		// l and r position: 0.005 m
		//VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005) //Uncomment this line to add measurement noise.
	};
};