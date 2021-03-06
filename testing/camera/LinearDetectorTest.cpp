#include <PSS/camera/LinearDetector.h>
#include <PSS/geometry/Rot3.h>
#include <PSS/geometry/Pose3.h>

#include <math.h>
#include <stdexcept>

#include <gtest/gtest.h>

#include <Eigen/Dense>


TEST(LinearDetectorTest, FOVConstructorTest) {
	PSS::Point3 position{ 1.0, 2.0, 3.0 };
	PSS::Rot3 rotation{ 1.0, 0.0, 0.0, 0.0 };
	PSS::Pose3 pose{ rotation, position };

	double fieldOfView{ 90.0 };
	double sensorWidth{ 0.1 };
	int resolution{ 30000 };
	double sensorVariance{ 0.001 };

	PSS::LinearDetector linDetector{ fieldOfView, sensorWidth, resolution, sensorVariance, pose };

	// check if the focal length was computed correctly
	double expectedFocalLength{ 0.05 };
	double delta{ std::abs(expectedFocalLength - linDetector.focalLength()) };
	double epsilon{ 0.000001 };
	ASSERT_TRUE(delta < epsilon);
}

TEST(LinearDetectorTest, GettersTest) {
	// create linear detector
	PSS::Point3 position{ 1.0, 2.0, 3.0 };
	PSS::Rot3 rotation{ 1.0, 0.0, 0.0, 0.0 };
	PSS::Pose3 pose{ rotation, position };

	double focalLength{ 0.075 };
	double sensorWidth{ 0.2 };
	int resolution{ 30000 };
	double centerOffset{ 0.1 };
	double sensorVariance{ 0.001 };

	PSS::LinearDetector linDetector{ focalLength, centerOffset, sensorWidth, resolution, sensorVariance, pose };

	// check getters
	ASSERT_EQ(linDetector.focalLength(), focalLength);
	ASSERT_EQ(linDetector.sensorWidth(), sensorWidth);
	ASSERT_EQ(linDetector.resolution(), resolution);
	ASSERT_EQ(linDetector.centerOffset(), centerOffset);

	ASSERT_EQ(linDetector.pose().translation(), position);
	ASSERT_TRUE(linDetector.pose().rotation().matrix().isApprox(rotation.matrix()));

	Eigen::Matrix<double, 2, 4> projMatrix;
	projMatrix << 0.075, 0, 0.1, -0.3750, 0, 0, 1, -3.0;
	ASSERT_TRUE(projMatrix.isApprox(linDetector.projectionMatrix()));
	ASSERT_TRUE(projMatrix.isApprox(linDetector.calibratedProjectionMatrix()));

	// test with calibratedPose
	PSS::Point3 calibratedPosition{ 2.0, 2.0, 3.0 };
	PSS::Pose3 calibratedPose{ rotation, calibratedPosition };
	PSS::LinearDetector calibratedLinDetector{ focalLength, centerOffset, sensorWidth, resolution, sensorVariance, pose, calibratedPose };

	Eigen::Matrix<double, 2, 4> calibratedProjMatrix;
	calibratedProjMatrix << 0.075, 0, 0.1, -0.450, 0, 0, 1, -3.0;
	ASSERT_TRUE(calibratedProjMatrix.isApprox(calibratedLinDetector.calibratedProjectionMatrix()));
}

TEST(LinearDetectorTest, ProjectionTest) {
	// create linear detector
	PSS::Point3 position{ 1.0, 2.0, 3.0 };
	PSS::Rot3 rotation{ 1.0, 0.0, 0.0, 0.0 };
	PSS::Pose3 pose{ rotation, position };

	double focalLength{ 0.075 };
	double sensorWidth{ 0.2 };
	int resolution{ 30000 };
	double centerOffset{ 0.1 };
	double sensorVariance{ 0.001 };

	PSS::LinearDetector linDetector{ focalLength, centerOffset, sensorWidth, resolution, sensorVariance, pose };

	// check projection
	PSS::Point3 point{ 1.0, 2.0, 4.0 };
	double expectedPoint{ 0.1 };
	bool addNoise{ false };
	double delta{ std::abs(linDetector.projectPoint(point, addNoise) - expectedPoint) };
	double epsilon{ 0.000001 };
	ASSERT_TRUE(delta < epsilon);

	// check safe projection
	// check point behind camera
	point(2, 0) = 2.0;
	ASSERT_THROW(linDetector.safeProjectPoint(point, addNoise), PSS::OutsideOfFieldOfView);

	// check point outside of sensor range
	point(2, 0) = 4.0;
	point(0, 0) = -10.0;
	ASSERT_THROW(linDetector.safeProjectPoint(point, addNoise), PSS::OutsideOfFieldOfView);
	point(0, 0) = 10.0;
	ASSERT_THROW(linDetector.safeProjectPoint(point, addNoise), PSS::OutsideOfFieldOfView);
}
