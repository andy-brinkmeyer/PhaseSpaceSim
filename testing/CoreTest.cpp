#include "Core/Core.h"
#include "Camera/Camera.h"

#include <gtest/gtest.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

#include <unordered_map>
#include <string>
#include <vector>


TEST(CoreTest, EstimationTest) {
	// create cameras
	double fieldOfView{ 120 };
	double sensorWidth{ 0.1 };

	gtsam::Point3 position1{ 0.0, 0.0, 0.0 };
	gtsam::Rot3 rotation1{ 1.0, 0.0, 0.0, 0.0 };
	gtsam::Pose3 pose1{ rotation1, position1 };
	PSS::Camera camera1{ fieldOfView, sensorWidth, pose1 };

	gtsam::Point3 position2{ -5.0, 1.0, 0.0 };
	gtsam::Rot3 rotation2{ 1, 0, 0, 0 };
	gtsam::Pose3 pose2{ rotation2, position2 };
	PSS::Camera camera2{ fieldOfView, sensorWidth, pose2 };

	gtsam::Point3 position3{ 0.0, 0.0, 10.0 };
	gtsam::Rot3 rotation3{ 0, 0, 1, 0 };
	gtsam::Pose3 pose3{ rotation2, position2 };
	PSS::Camera camera3{ fieldOfView, sensorWidth, pose3 };
	
	// create core
	PSS::CameraMap cameraMap{
		{ "Cam1", camera1 },
		{ "Cam2", camera2 },
		{ "Cam3", camera3 }
	};
	PSS::Core core{ cameraMap };

	// estimate point
	gtsam::Point3 knownPoint{ -2.5, 0.5, 5.0 };
	std::vector<std::string> cameraList;
	cameraList.push_back("Cam1");
	cameraList.push_back("Cam2");
	cameraList.push_back("Cam3");

	gtsam::Point3 estimatedPoint{ core.estimateFromCameras(knownPoint, cameraList) };
	ASSERT_TRUE(estimatedPoint.isApprox(knownPoint));

	// test for underdetermined system
	cameraList.pop_back();
	cameraList.pop_back();
	ASSERT_THROW(core.estimateFromCameras(knownPoint, cameraList), PSS::UnderdeterminedSystem);
}