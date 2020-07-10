#pragma once

#include "LinearDetector.h"

#include <gtsam/geometry/Pose3.h>


namespace PSS {
	class Camera {
	public:
		// constructors
		Camera(LinearDetector& horizontalDetector, LinearDetector& verticalDetector);
		Camera(double fieldOfView, double sensorWidth, gtsam::Pose3& pose);
		Camera(double fieldOfView, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose);
		Camera(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose);
		Camera(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose);

		// getters
		LinearDetector& horizontalDetector();
		LinearDetector& verticalDetector();

	private:
		LinearDetector mHorizontalDetector;
		LinearDetector mVerticalDetector;

		// helper functions
		gtsam::Pose3 rotateToVertical(gtsam::Pose3& pose);
	};
}