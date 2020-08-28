#define _USE_MATH_DEFINES

#include "Camera.h"
#include "LinearDetector.h"
#include "../geometry/Pose3.h"
#include "../geometry/Rot3.h"

#include <Eigen/Core>

#include <cmath>
#include <exception>


namespace PSS {
	// constructors
	Camera::Camera(double fieldOfView, double sensorWidth, int resolution, double sensorVariance, const Pose3& pose)
		: mHorizontalDetector{ fieldOfView, sensorWidth, resolution, sensorVariance, pose }
		, mVerticalDetector{ fieldOfView, sensorWidth, resolution, sensorVariance, rotateToVertical(pose) } { }

	Camera::Camera(double fieldOfView, double sensorWidth, int resolution, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose)
		: mHorizontalDetector{ fieldOfView, sensorWidth, resolution, sensorVariance, pose, calibratedPose }
		, mVerticalDetector{ fieldOfView, sensorWidth, resolution, sensorVariance, rotateToVertical(pose), rotateToVertical(calibratedPose) } { }

	Camera::Camera(double focalLength, double centerOffset, double sensorWidth, int resolution, double sensorVariance, const Pose3& pose)
		: mHorizontalDetector{ focalLength, centerOffset, sensorWidth, resolution, sensorVariance, pose }
		, mVerticalDetector{ focalLength, centerOffset, sensorWidth, resolution, sensorVariance, rotateToVertical(pose) } { }

	Camera::Camera(double focalLength, double centerOffset, double sensorWidth, int resolution, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose)
		: mHorizontalDetector{ focalLength, centerOffset, sensorWidth, resolution, sensorVariance, pose, calibratedPose }
		, mVerticalDetector{ focalLength, centerOffset, sensorWidth, resolution, sensorVariance, rotateToVertical(pose), rotateToVertical(calibratedPose) } { }

	// getters
	LinearDetector& Camera::horizontalDetector() { return mHorizontalDetector; }
	LinearDetector& Camera::verticalDetector() { return mVerticalDetector; }

	// estimation
	Eigen::Matrix<double, Eigen::Dynamic, 4> Camera::getEstimationEquations(const Point3& point, bool addSensorNoise) {
		Eigen::Matrix<double, Eigen::Dynamic, 4> equations;

		// get equation for horizontal detector
		Eigen::Matrix<double, 1, 4> tmpMat;
		try {
			tmpMat << mHorizontalDetector.getEstimationEquation(point, addSensorNoise);	// store in tmp variable to see if an error is thrown
			equations.conservativeResize(equations.rows() + 1, 4);
			equations.row(equations.rows() - 1) = tmpMat;
		}
		catch (const OutsideOfFieldOfView&) { }

		// get equation for vertical detector
		try {
			tmpMat << mVerticalDetector.getEstimationEquation(point, addSensorNoise);	// store in tmp variable to see if an error is thrown
			equations.conservativeResize(equations.rows() + 1, 4);
			equations.row(equations.rows() - 1) = tmpMat;
		}
		catch (const OutsideOfFieldOfView&) { }

		// throw exception if point is not visible
		if (equations.rows() > 0) {
			return equations;
		}
		else {
			throw OutsideOfFieldOfView("Point not visible by camera, no equations available.");
		}
	}

	// helper functions
	Pose3 Camera::rotateToVertical(const Pose3& pose) {
		Rot3 rot{ M_SQRT1_2, 0, 0, -M_SQRT1_2 };
		RotationMatrix3 verticalRotMat{ pose.rotation().matrix() * rot.matrix() };
		Rot3 verticalRot{ verticalRotMat };
		Pose3 rotatedPose{ verticalRot, pose.translation() };
		return rotatedPose;
	}
}