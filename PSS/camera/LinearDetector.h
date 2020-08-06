#pragma once

#include "../geometry/Pose3.h"

#include <Eigen/Core>

#include <random>
#include <exception>


namespace PSS {
	/**
	 * \brief Class representing a Linear Detector based on the pinhole camera model.
	 *
	 * A linear detector is treated as a pinhole camera where the sensor is reduced to only one dimension. 
	 * This is done by removing the row of the projection matrix that would correspond to the second sensor dimension. 
	 * The resulting LinearDetector::ProjectionMatrix is of dimension 2x4.
	*/
	class LinearDetector {
	public:
		// typedefs
		/**
		 * \brief 2x4 Projection Matrix of a LinearDetector.
		 *
		 * The projection matrix is of the form:
		 * \f$ P = \begin{bmatrix} f & 0 & c \\ 0 & 0 & 1 \end{bmatrix} 
		 * \mathbf{R} \left[ \mathbf{I}  \mid  -\mathbf{x} \right] \f$
		 * 
		 * Where \f$ f \f$ is the focal length, \f$ c \f$ is the sensor center offset, \f$ \mathbf{R} \f$ is the rotation matrix that maps a vector from the local frame to the detector frame
		 * and \f$ \mathbf{x} \f$ is the linear detectors position in the local frame.
		*/
		typedef Eigen::Matrix<double, 2, 4> ProjectionMatrix;

	private:
		// intrinsics
		double mFocalLength;
		double mSensorWidth;
		double mSensorVariance;
		double mCenterOffset;

		// pose
		Pose3 mPose;
		Pose3 mCalibratedPose;

		// projection
		ProjectionMatrix mProjectionMatrix;
		ProjectionMatrix mCalibratedProjectionMatrix;

		/**
		 * \brief Utility function used to compute the LinearDetector::ProjectionMatrix.
		*/
		ProjectionMatrix computeProjectionMatrix(double focalLength, double centerOffset, double sensorWidth, Pose3& pose);

		// estimation
		Eigen::Matrix<double, 1, 4> mC1;	/**< First row of LinearDetector::mProjectionMatrix matrix for quick access */
		Eigen::Matrix<double, 1, 4> mC2;	/**< Second row of LinearDetector::mProjectionMatrix matrix for quick access */

		// gaussian noise
		std::random_device mRandomDevice;
		std::mt19937 mRandomGenerator;
		std::normal_distribution<double> mNormalDistribution;

	public:
		// constructor
		/**
		 * \brief Default consructor.
		*/
		LinearDetector(const LinearDetector& linearDetector);

		/**
		 * \brief Convenience constructor from FoV.
		 * 
		 * It is assumed that the camera is perfectly calibrated.
		 *
		 * \param fieldOfView The field of view of the linear detector in degrees.
		 * \param sensorWidth Width of the sensor in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0, 
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the linear detector with respect to the local frame.
		*/
		LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose);

		/**
		 * \brief Convenience constructor from FoV.
		 *
		 * Use this constructor when the calibrated camera pose differs from the true one. The projection will be performed using the true
		 * camera pose while for estimation the calibrated LinearDetector::ProjectionMatrix is used.
		 *
		 * \param fieldOfView The field of view of the linear detector in degrees.
		 * \param sensorWidth Width of the sensor in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the linear detector with respect to the local frame.
		 * \param calibratedPose Calibrated pose of the linear detector with respect to the local frame.
		*/
		LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);

		/**
		 * \brief Constructor from linear detector properties.
		 *
		 * It is assumed that the camera is perfectly calibrated.
		 *
		 * \param focalLength The focal length of the linear detector in metres.
		 * \param centerOffset Offset of the sensor center in metres. Read about Camera Pinhole Model for more details about the offset.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the linear detector with respect to the local frame.
		*/
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose);

		/**
		 * \brief Constructor from linear detector properties.
		 *
		 * Use this constructor when the calibrated camera pose differs from the true one. The projection will be performed using the true
		 * camera pose while for estimation the calibrated LinearDetector::ProjectionMatrix is used.
		 *
		 * \param focalLength The focal length of the linear detector in metres.
		 * \param centerOffset Offset of the sensor center in metres. Read about Camera Pinhole Model for more details about the offset.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the linear detector with respect to the local frame.
		 * \param calibratedPose Calibrated pose of the linear detector with respect to the local frame.
		*/
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);

		// getters
		double focalLength() const; /**< Returns the focal length. */
		double sensorWidth() const; /**< Returns the sensor width. */
		double sensorVariance() const; /**< Returns the sensor variance. */
		double centerOffset() const; /**< Returns the center offset. */
		const Pose3& pose() const; /**< Returns the true pose. */
		const Pose3& calibratedPose() const; /**< Returns the calibrated pose. */
		const ProjectionMatrix& projectionMatrix() const; /**< Returns the true projection matrix. */
		const ProjectionMatrix& calibratedProjectionMatrix() const; /**< Returns the calibrated projection matrix. */

		// projection
		/**
		 * \brief Project a point onto the linear detector sensor.
		 *
		 * This function is equivalent to right-multiplying the true projection matrix with the homogeneous equivalent of the point
		 * and normalizing the homogeneous result. No boundary checking is performed. For this task use LinearDetector::safeProjectPoint.
		 *
		 * \param point The point in the local frame to project.
		 * \param addNoise Add zero-mean Gaussian noise to the true measurement.
		*/
		double projectPoint(const Point3 &point, bool addNoise = true);

		/**
		 * \brief Check if the point is visible by the linear detector and project it onto the sensor.
		 *
		 * \param point The point in the local frame to project.
		 * \param addNoise Add zero-mean Gaussian noise to the true measurement.
		*/
		double safeProjectPoint(const Point3& point, bool addNoise = true);

		// estimation
		Eigen::Matrix<double, 1, 4> getEstimationEquation(const Point3& point, bool addSensorNoise = true);
	};

	// custom exception
	class OutsideOfFieldOfView : public std::exception {
		std::string mMsg;

	public:
		OutsideOfFieldOfView(const std::string& msg);

		virtual const char* what() const throw ();
	};
}
