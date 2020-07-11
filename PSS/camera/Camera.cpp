#define _USE_MATH_DEFINES

#include "Camera.h"

#include "LinearDetector.h"

#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

#include <cmath>
#include <exception>


namespace PSS {
	// constructors
	Camera::Camera(LinearDetector& horizontalDetector, LinearDetector& verticalDetector) : mHorizontalDetector{ horizontalDetector }, mVerticalDetector{ verticalDetector } { }

	Camera::Camera(double fieldOfView, double sensorWidth, gtsam::Pose3& pose) : mHorizontalDetector{ fieldOfView, sensorWidth, pose }, mVerticalDetector{ fieldOfView, sensorWidth, rotateToVertical(pose) } { }

	Camera::Camera(double fieldOfView, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose) 
		: mHorizontalDetector{ fieldOfView, sensorWidth, pose, calibratedPose }, mVerticalDetector{ fieldOfView, sensorWidth, rotateToVertical(pose), rotateToVertical(calibratedPose) } { }

	Camera::Camera(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose)
		: mHorizontalDetector{ focalLength, centerOffset, sensorWidth, pose },
		  mVerticalDetector{ focalLength, centerOffset, sensorWidth, rotateToVertical(pose) } { }

	Camera::Camera(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose) 
		: mHorizontalDetector{ focalLength, centerOffset, sensorWidth, pose, calibratedPose },
		  mVerticalDetector{ focalLength, centerOffset, sensorWidth, rotateToVertical(pose), rotateToVertical(calibratedPose) } { }

	// getters
	LinearDetector& Camera::horizontalDetector() { return mHorizontalDetector; }
	LinearDetector& Camera::verticalDetector() { return mVerticalDetector; }

	// estimation
	Eigen::Matrix<double, Eigen::Dynamic, 4> Camera::getEstimationEquations(gtsam::Point3& point) {
		Eigen::Matrix<double, Eigen::Dynamic, 4> equations;

		// get equation for horizontal detector
		Eigen::Matrix<double, 1, 4> tmpMat;
		try {
			tmpMat << mHorizontalDetector.getEstimationEquation(point);	// store in tmp variable to see if an error is thrown
			equations.conservativeResize(equations.rows() + 1, 4);
			equations.row(equations.rows() - 1) = tmpMat;
		}
		catch (const std::domain_error& e) { }

		// get equation for vertical detector
		try {
			tmpMat << mVerticalDetector.getEstimationEquation(point);	// store in tmp variable to see if an error is thrown
			equations.conservativeResize(equations.rows() + 1, 4);
			equations.row(equations.rows() - 1) = tmpMat;
		}
		catch (const std::domain_error& e) { }

		// throw exception if point is not visible
		if (equations.rows() > 0) {
			return equations;
		}
		else {
			throw std::domain_error("Point not visible by camera, no equations available.");
		}
	}

	// helper functions
	gtsam::Pose3 Camera::rotateToVertical(gtsam::Pose3& pose) {
		gtsam::Rot3 rot{ M_SQRT1_2, 0, 0, -M_SQRT1_2 };
		gtsam::Pose3 rotatedPose{ rot * pose.rotation(), pose.translation() };
		return rotatedPose;
	}
}