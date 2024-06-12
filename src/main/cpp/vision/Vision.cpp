#include "vision/Vision.h"

Vision::Vision()
{
	if (frc::RobotBase::IsSimulation())
	{
		simVision.AddAprilTags(frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo));

		camProp.SetCalibration(640, 480, 70.0_deg);
		camProp.SetCalibError(0.25, 0.08);
		camProp.SetFPS(55_Hz);
		camProp.SetAvgLatency(16_ms);
		camProp.SetLatencyStdDev(3_ms);

		pi11CamSim = photon::PhotonCameraSim{&pi11Cam, camProp};
		pi12CamSim = photon::PhotonCameraSim{&pi12Cam, camProp};
		simVision.AddCamera(&pi11CamSim, kCam11ToRobot);
		simVision.AddCamera(&pi12CamSim, kCam12ToRobot);
	}

	pi11PhotonEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);
	pi12PhotonEstimator.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);
}

void Vision::update(frc::DifferentialDrivePoseEstimator& poseEstimator)
{
	updatePose(pi11PhotonEstimator.Update(pi11Cam.GetLatestResult()), poseEstimator);
	updatePose(pi12PhotonEstimator.Update(pi12Cam.GetLatestResult()), poseEstimator);
	// return PiPoseEstimations{est11Result, est12Result};
}

void Vision::updateSim(frc::DifferentialDrivePoseEstimator& poseEstimator)
{
	frc::Pose2d position = poseEstimator.GetEstimatedPosition();
	simVision.Update(position);
    simVision.GetDebugField().GetObject("EstimatedRobot")->SetPose(position);

    photon::PhotonPipelineResult latest11Result = pi11Cam.GetLatestResult();
	auto est11Result = pi11PhotonEstimator.Update(latest11Result);
	updatePose(est11Result, poseEstimator);

    photon::PhotonPipelineResult latest12Result = pi12Cam.GetLatestResult();
	auto est12Result = pi12PhotonEstimator.Update(latest12Result);
	updatePose(est12Result, poseEstimator);

	//Debugging
	if (est11Result)
	{
		frc::SmartDashboard::PutBoolean("Photon 11 result: ", true);
		frc::SmartDashboard::PutNumber("Photon 11 x: ", (double)est11Result.value().estimatedPose.X());
		frc::SmartDashboard::PutNumber("Photon 11 y: ", (double)est11Result.value().estimatedPose.Y());
		frc::SmartDashboard::PutNumber("Photon 11 z: ", (double)est11Result.value().estimatedPose.Z());
	}
	else
	{
		frc::SmartDashboard::PutBoolean("Photon 11 result: ", false);
		frc::SmartDashboard::PutNumber("Photon 11 x: ", -1);
		frc::SmartDashboard::PutNumber("Photon 11 y: ", -1);
		frc::SmartDashboard::PutNumber("Photon 11 z: ", -1);
	}
	if (est12Result)
	{
		frc::SmartDashboard::PutBoolean("Photon 12 result: ", true);
		frc::SmartDashboard::PutNumber("Photon 12 x: ", (double)est12Result.value().estimatedPose.X());
		frc::SmartDashboard::PutNumber("Photon 12 y: ", (double)est12Result.value().estimatedPose.Y());
		frc::SmartDashboard::PutNumber("Photon 12 z: ", (double)est12Result.value().estimatedPose.Z());
	}
	else
	{
		frc::SmartDashboard::PutBoolean("Photon 12 result: ", false);
		frc::SmartDashboard::PutNumber("Photon 12 x: ", -1);
		frc::SmartDashboard::PutNumber("Photon 12 y: ", -1);
		frc::SmartDashboard::PutNumber("Photon 12 z: ", -1);
	}
	
	// return PiPoseEstimations{est11Result, est12Result};
}

std::vector<int64_t> Vision::getCurrentTargetIDs()
{
	std::vector<int64_t> currentTargetIDs;
	std::span<const photon::PhotonTrackedTarget> targets11 = pi11Cam.GetLatestResult().GetTargets();
	std::span<const photon::PhotonTrackedTarget> targets12 = pi12Cam.GetLatestResult().GetTargets();

	for (auto i = targets11.begin(); i != targets11.end(); ++i)
	{
		currentTargetIDs.push_back(i->GetFiducialId());
	}
	for (auto i = targets11.begin(); i != targets11.end(); ++i)
	{
		currentTargetIDs.push_back(i->GetFiducialId());
	}
	return currentTargetIDs;
}

void Vision::updatePose(std::optional<photon::EstimatedRobotPose> estPose, frc::DifferentialDrivePoseEstimator& poseEstimator)
{
	if (estPose) poseEstimator.AddVisionMeasurement(estPose.value().estimatedPose.ToPose2d(), estPose.value().timestamp);
}