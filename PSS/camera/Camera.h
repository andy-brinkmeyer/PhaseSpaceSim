#pragma once

#include "LinearDetector.h"
#include "../geometry/Pose3.h"

#include <Eigen/Core>


namespace PSS {
	/**
	 * \brief Class that represents a PhaseSpace camera made up of two linear detectors.
	 *
	 * The PhaseSpace system uses cameras that are made up of two independent optical linear detectors that are rotated against each other.
	 * Here the rotation is assumed to be 90 degrees.
	*/
	class Camera {
		LinearDetector mHorizontalDetector;
		LinearDetector mVerticalDetector;

		// helper functions
		Pose3 rotateToVertical(const Pose3& pose); // rotates a pose 90 degrees around the cameras z-axis

	public:
		// constructors
		/**
		 * \brief Convenience constructor from FoV. The camera is perfectly calibrated.
		 *
		 * It is assumed that both linear detectors have the same intrinsic properties, i.e. the same focal length, sensor width, center offset and sensor variance. 
		 * The center of the sensor is assumed to be in its middle.
		 *
		 * \param fieldOfView Field of view of the linear detectors in degrees.
		 * \param sensorWidth Width of the sensors in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the camera. The camera faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
		*/
		Camera(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose);

		/**
		 * \brief Convenience constructor from FoV. The cameras pose is not perfectly calibrated.
		 *
		 * It is assumed that both linear detectors have the same intrinsic properties, i.e. the same focal length, sensor width, center offset and sensor variance.
		 * The center of the sensor is assumed to be in its middle.
		 *
		 * \param fieldOfView Field of view of the linear detectors in degrees.
		 * \param sensorWidth Width of the sensors in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the camera. The camera faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
		 * \param calibratedPose Calibrated pose of the camera. The camera faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
		*/
		Camera(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);

		/**
		 * \brief Constructor from the linear detectors properties. The camera is perfectly calibrated.
		 *
		 * It is assumed that both linear detectors have the same intrinsic properties, i.e. the same focal length, sensor width, center offset and sensor variance.
		 *
		 * \param focalLength The focal length of the linear detectors in metres.
		 * \param centerOffset Offset of the sensors center in metres. Read about Camera Pinhole Model for more details about the offset.
		 * \param sensorWidth Width of the sensors in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the camera. The camera faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
		*/
		Camera(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose);

		/**
		 * \brief Constructor from the linear detectors properties. The camera is not perfectly calibrated.
		 *
		 * It is assumed that both linear detectors have the same intrinsic properties, i.e. the same focal length, sensor width, center offset and sensor variance.
		 *
		 * \param focalLength The focal length of the linear detectors in metres.
		 * \param centerOffset Offset of the sensors center in metres. Read about Camera Pinhole Model for more details about the offset.
		 * \param sensorWidth Width of the sensors in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the camera. The camera faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
		 * \param calibratedPose Calibrated pose of the camera. The camera faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
		*/
		Camera(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);

		// getters
		const LinearDetector& horizontalDetector() const; /**< \brief Returns a reference to the horizonal LinearDetector. */
		const LinearDetector& verticalDetector() const; /**< \brief Returns a reference to the vertical LinearDetector. */

		// estimation
		/**
		 * \brief Returns the estimation equation corresponding to this cameras linear detectors.
		 *
		 * See LinearDetector::getEstimationEquation for more information. 0-2 equations will be returned depending on if the point
		 * is visible from the individual linear detectors.
		 *
		 * \param point True position of the point to estimate.
		 * \param addSensorNoise Add zero-mean Gaussian noise on the true sensor measurements.
		*/
		Eigen::Matrix<double, Eigen::Dynamic, 4> getEstimationEquations(const Point3& point, bool addSensorNoise = true);
	};
}