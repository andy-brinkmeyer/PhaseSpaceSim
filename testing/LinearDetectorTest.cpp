#include <camera/LinearDetector.h>

#include <math.h>
#include <stdexcept>

#include <gtest/gtest.h>

#include <Eigen/Eigen>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>


TEST(LinearDetectorTest, GettersTest) {
	// create linear detector
	gtsam::Point3 position{ 1.0, 2.0, 3.0 };
	gtsam::Rot3 rotation{ 1.0, 0.0, 0.0, 0.0 };
	gtsam::Pose3 pose{ rotation, position };

	double focalLength{ 0.075 };
	double sensorWidth{ 0.2 };
	double centerOffset{ 0.1 };

	PSS::LinearDetector linDetector{ focalLength, centerOffset, sensorWidth, pose };

	// check getters
	ASSERT_EQ(linDetector.focalLength(), focalLength);
	ASSERT_EQ(linDetector.centerOffset(), centerOffset);

	ASSERT_EQ(linDetector.pose().translation(), position);
	ASSERT_TRUE(linDetector.pose().rotation().equals(rotation));

	Eigen::Matrix<double, 2, 4> projMatrix;
	projMatrix << 0.075, 0, 0.1, -0.3750, 0, 0, 1, -3.0;
	ASSERT_TRUE(projMatrix.isApprox(linDetector.projectionMatrix()));
}

TEST(LinearDetectorTest, ProjectionTest) {
	// create linear detector
	gtsam::Point3 position{ 1.0, 2.0, 3.0 };
	gtsam::Rot3 rotation{ 1.0, 0.0, 0.0, 0.0 };
	gtsam::Pose3 pose{ rotation, position };

	double focalLength{ 0.075 };
	double sensorWidth{ 0.2 };
	double centerOffset{ 0.1 };

	PSS::LinearDetector linDetector{ focalLength, centerOffset, sensorWidth, pose };

	// check projection
	gtsam::Point3 point{ 1.0, 2.0, 4.0 };
	double expectedPoint{ 0.1 };
	double delta{ std::abs(linDetector.projectPoint(point) - expectedPoint) };
	double epsilon{ 0.000001 };
	ASSERT_TRUE(delta < epsilon);

	// check safe projection
	point(2, 0) = 2.0;
	ASSERT_THROW(linDetector.safeProjectPoint(point), std::domain_error);
}
