#pragma once

#include "LinearDetector.h"

#include <gtsam/geometry/Pose3.h>


namespace PSS {
	class Camera {
	public:
		Camera(LinearDetector& horizontalDetector, LinearDetector& verticalDetector);
		Camera(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose);
		Camera(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose);

	private:
		LinearDetector mHorizontalDetector;
		LinearDetector mVerticalDetector;

		// helper functions
		gtsam::Pose3 rotateToVertical(gtsam::Pose3& pose);
	};
}