#define _USE_MATH_DEFINES

#include <PSS/camera/Camera.h>
#include <PSS/camera/LinearDetector.h>
#include <PSS/geometry/Rot3.h>
#include <PSS/geometry/Pose3.h>

#include <gtest/gtest.h>

#include <cmath>


TEST(CameraTest, ConstructorTest) {
	double fieldOfView{ 90 };
	double sensorWidth{ 0.1 };
	double sensorVariance{ 0.001 };

	PSS::Point3 position{ 1.0, 2.0, 3.0 };
	PSS::Rot3 rotation{ 1.0, 0.0, 0.0, 0.0 };
	PSS::Pose3 pose{ rotation, position };

	PSS::Camera* camera = new PSS::Camera{ fieldOfView, sensorWidth, sensorVariance, pose };

	// check for horizontal detector
	const PSS::LinearDetector* horizontalDetector = &camera->horizontalDetector();
	double expectedFocalLength{ 0.05 };
	double delta{ std::abs(expectedFocalLength - horizontalDetector->focalLength()) };
	double epsilon{ 0.000001 };
	ASSERT_TRUE(delta < epsilon);

	ASSERT_EQ(horizontalDetector->pose().translation(), position);
	ASSERT_TRUE(horizontalDetector->pose().rotation().matrix().isApprox(rotation.matrix()));

	// check for vertical detector
	const PSS::LinearDetector* verticalDetector = &camera->verticalDetector();
	delta = std::abs(expectedFocalLength - verticalDetector->focalLength());
	ASSERT_TRUE(delta < epsilon);

	ASSERT_EQ(verticalDetector->pose().translation(), position);

	PSS::Rot3 expectedRot{ M_SQRT1_2, 0, 0, -M_SQRT1_2 }; // the vertical detector is rotated by -90 degress around the z-axis
	ASSERT_TRUE(verticalDetector->pose().rotation().matrix().isApprox(expectedRot.matrix()));
}