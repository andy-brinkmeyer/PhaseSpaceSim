#include "SimulationContextTest.h"
#include <core/Core.h>
#include <core/SimulationContext.h>
#include <camera/Camera.h>
#include <camera/LinearDetector.h>
#include <core/SimulationContext.h>

#include <gtest/gtest.h>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>

#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>


// create fixture
class CoreTestWithContext : public SimulationContextFixture { };

TEST_F(CoreTestWithContext, ConstructFromSimContextTest) {
	// create simulation context
	PSS::SimulationContext simContext{ metaDataPath, measurementsPath, outputPath };

	// create the core object
	PSS::Core core{ simContext };

	// check if the cameras are set up correctly
	ASSERT_EQ(core.cameras().size(), numCameras);
	PSS::CameraMap::const_iterator foundCamera{ core.cameras().find(camera2ID) };
	const PSS::Camera& cam2{ foundCamera->second };
	const PSS::LinearDetector& horizontalDet{ cam2.horizontalDetector() };
	ASSERT_TRUE(horizontalDet.pose().translation().isApprox(camera2Pos)); // check position
	ASSERT_TRUE(horizontalDet.pose().rotation().matrix().isApprox(camera2Rot.matrix())); // check rotation

	// check focal length
	double focalLength{ sensorWidth / (2 * std::tan(0.5 * (fieldOfView * M_PI / 180))) };
	double delta{ std::abs(horizontalDet.focalLength() - focalLength) };
	double epsilon{ 0.000001 };
	ASSERT_TRUE(delta < epsilon);

	// check sensor width and variance
	delta = std::abs(horizontalDet.sensorWidth() - sensorWidth);
	ASSERT_TRUE(delta < epsilon);
	delta = std::abs(horizontalDet.sensorVariance() - sensorVariance);
	ASSERT_TRUE(delta < epsilon);

	// if all the above are true then we assume that the other cameras have been properly intialized as well
}

TEST(CoreTest, EstimationTest) {
	// create cameras
	double fieldOfView{ 120 };
	double sensorWidth{ 0.1 };
	double sensorVariance{ 0.001 };

	gtsam::Point3 position1{ 0.0, 0.0, 0.0 };
	gtsam::Rot3 rotation1{ 1.0, 0.0, 0.0, 0.0 };
	gtsam::Pose3 pose1{ rotation1, position1 };
	PSS::Camera* camera1 = new PSS::Camera{ fieldOfView, sensorWidth, sensorVariance, pose1 };

	gtsam::Point3 position2{ -5.0, 1.0, 0.0 };
	gtsam::Rot3 rotation2{ 1, 0, 0, 0 };
	gtsam::Pose3 pose2{ rotation2, position2 };
	PSS::Camera* camera2 = new PSS::Camera{ fieldOfView, sensorWidth, sensorVariance, pose2 };

	gtsam::Point3 position3{ 0.0, 0.0, 10.0 };
	gtsam::Rot3 rotation3{ 0, 0, 1, 0 };
	gtsam::Pose3 pose3{ rotation3, position3 };
	PSS::Camera* camera3 = new PSS::Camera{ fieldOfView, sensorWidth, sensorVariance, pose3 };
	
	// create core
	PSS::CameraMap* cameraMap = new PSS::CameraMap{ };
	cameraMap->insert({ "Cam1", *camera1 });
	cameraMap->insert({ "Cam2", *camera2 });
	cameraMap->insert({ "Cam3", *camera3 });

	PSS::Core core{ *cameraMap };

	// estimate point
	gtsam::Point3 knownPoint{ -2.5, 0.5, 5.0 };
	std::vector<std::string> cameraList;
	cameraList.push_back("Cam1");
	cameraList.push_back("Cam2");
	cameraList.push_back("Cam3");

	bool addSensorNoise{ false };
	gtsam::Point3 estimatedPoint{ core.estimateFromCameras(knownPoint, cameraList, addSensorNoise) };
	ASSERT_TRUE(estimatedPoint.isApprox(knownPoint));

	// test for underdetermined system
	cameraList.pop_back();
	cameraList.pop_back();
	ASSERT_THROW(core.estimateFromCameras(knownPoint, cameraList), PSS::UnderdeterminedSystem);
}

/*
// This one is just for quick testing, will be removed once the proper tests are written.
TEST(CoreTest, SimulateCameraOnlyTest) {
	PSS::SimulationContext simContext{ "meta.json", "measurements.csv", "out.csv" };
	PSS::Core core{ simContext };
	Eigen::Matrix3d mat{ core.cameras().find("Camera_1")->second.horizontalDetector().pose().rotation().matrix() };
	core.simulateCameraOnly(simContext);
}
*/