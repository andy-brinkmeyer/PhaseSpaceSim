#pragma once

#include "LinearDetector.h"

#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>


namespace PSS {
	class Camera {
	public:
		// constructors
		Camera();
		Camera(LinearDetector& horizontalDetector, LinearDetector& verticalDetector);
		Camera(double fieldOfView, double sensorWidth, const gtsam::Pose3& pose);
		Camera(double fieldOfView, double sensorWidth, const gtsam::Pose3& pose, const gtsam::Pose3& calibratedPose);
		Camera(double focalLength, double centerOffset, double sensorWidth, const gtsam::Pose3& pose);
		Camera(double focalLength, double centerOffset, double sensorWidth, const gtsam::Pose3& pose, const gtsam::Pose3& calibratedPose);

		// getters
		LinearDetector& horizontalDetector();
		LinearDetector& verticalDetector();

		// estimation
		Eigen::Matrix<double, Eigen::Dynamic, 4> getEstimationEquations(gtsam::Point3& point);

	private:
		LinearDetector mHorizontalDetector;
		LinearDetector mVerticalDetector;

		// helper functions
		gtsam::Pose3 rotateToVertical(const gtsam::Pose3& pose);
	};
}