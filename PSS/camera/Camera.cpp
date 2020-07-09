#define _USE_MATH_DEFINES

#include "Camera.h"

#include <cmath>

#include "LinearDetector.h"

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>


namespace PSS {
	// constructors
	Camera::Camera(LinearDetector& horizontalDetector, LinearDetector& verticalDetector) : mHorizontalDetector{ horizontalDetector }, mVerticalDetector{ verticalDetector } { }

	Camera::Camera(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose)
		: mHorizontalDetector{ focalLength, centerOffset, sensorWidth, pose },
		  mVerticalDetector{ focalLength, centerOffset, sensorWidth, rotateToVertical(pose) } { }

	Camera::Camera(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose) 
		: mHorizontalDetector{ focalLength, centerOffset, sensorWidth, pose, calibratedPose },
		  mVerticalDetector{ focalLength, centerOffset, sensorWidth, rotateToVertical(pose), rotateToVertical(calibratedPose) } { }

	// helper functions
	gtsam::Pose3 Camera::rotateToVertical(gtsam::Pose3& pose) {
		gtsam::Rot3 rot{ M_SQRT1_2, 0, 0, -M_SQRT1_2 };
		gtsam::Pose3 rotatedPose{ rot * pose.rotation(), pose.translation() };
		return rotatedPose;
	}
}