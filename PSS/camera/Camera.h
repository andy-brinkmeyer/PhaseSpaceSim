#pragma once

#include "LinearDetector.h"
#include "../geometry/Pose3.h"

#include <Eigen/Core>


namespace PSS {
	class Camera {
		LinearDetector mHorizontalDetector;
		LinearDetector mVerticalDetector;

		// helper functions
		Pose3 rotateToVertical(const Pose3& pose);

	public:
		// constructors
		Camera(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose);
		Camera(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);
		Camera(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose);
		Camera(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);

		// getters
		const LinearDetector& horizontalDetector() const;
		const LinearDetector& verticalDetector() const;

		// estimation
		Eigen::Matrix<double, Eigen::Dynamic, 4> getEstimationEquations(const Point3& point, bool addSensorNoise = true);
	};
}