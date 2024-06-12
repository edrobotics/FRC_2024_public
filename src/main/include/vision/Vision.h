#pragma once

#include <frc/geometry/CoordinateSystem.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/constants.h>

#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/RobotBase.h>

#include <photon/simulation/VisionSystemSim.h>
#include <photon/PhotonUtils.h>
#include <photon/estimation/TargetModel.h>
#include <photon/PhotonPoseEstimator.h>
#include <photon/simulation/VisionTargetSim.h>

#include "Constants.h"

class Vision
{
    public:

    Vision();

	/**
	 * Call every loop to update vision and the pose estimator
	 * @param poseEstimator The DifferentialDrivePoseEstimator to potentially update with vision data
	*/
    void update(frc::DifferentialDrivePoseEstimator& poseEstimator);
	/**
	 * For simulation, call every loop
	 * @param poseEstimator The DifferentialDrivePoseEstimator to potentially update with vision data, and get data from
	*/
    void updateSim(frc::DifferentialDrivePoseEstimator& poseEstimator);

	/**
	 * Returns the Fiscial ID of every current target, if any. Use for debugging and logging.
	*/
	std::vector<int64_t> getCurrentTargetIDs();

	private:
	/**
	 * Updating the poseEstimator with the estimated vision pose
	*/
	void updatePose(std::optional<photon::EstimatedRobotPose> estPose, frc::DifferentialDrivePoseEstimator& poseEstimator);

  	// The position and rotation of the cameras
	const frc::Transform3d kCam11ToRobot{VisionConstants::kCamera11X, VisionConstants::kCamera11Y, VisionConstants::kCamera11Z, 
										 frc::Rotation3d{VisionConstants::kCamera11Roll, VisionConstants::kCamera11Pitch, VisionConstants::kCamera11Yaw}};
	const frc::Transform3d kCam12ToRobot{VisionConstants::kCamera12X, VisionConstants::kCamera12Y, VisionConstants::kCamera12Z, 
										 frc::Rotation3d{VisionConstants::kCamera12Roll, VisionConstants::kCamera12Pitch, VisionConstants::kCamera12Yaw}};

	// Cameras
	photon::PhotonCamera pi11Cam{"Pi11_OV9281"};
	photon::PhotonCamera pi12Cam{"Pi12_OV9281"};

	// The pose estimators
	photon::PhotonPoseEstimator pi11PhotonEstimator{frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo), photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, kCam11ToRobot};
	photon::PhotonPoseEstimator pi12PhotonEstimator{frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo), photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, kCam12ToRobot};

	// Simulation
    photon::VisionSystemSim simVision{"main"};

	photon::SimCameraProperties camProp{};

	photon::PhotonCameraSim pi11CamSim{&pi11Cam, camProp};
	photon::PhotonCameraSim pi12CamSim{&pi11Cam, camProp};
};