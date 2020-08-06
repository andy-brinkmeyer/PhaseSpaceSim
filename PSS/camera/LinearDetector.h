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
		 *
		 * \param linearDetector LinearDetector instance.
		*/
		LinearDetector(const LinearDetector& linearDetector);

		/**
		 * \brief Convenience constructor from FoV.
		 * 
		 * It is assumed that the camera is perfectly calibrated. 
		 * 
		 * The sensors center is assumed to be in its middle.
		 *
		 * \param fieldOfView The field of view of the linear detector in degrees.
		 * \param sensorWidth Width of the sensor in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0, 
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the linear detector with respect to the local frame. 
		 *			The detector faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
		*/
		LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose);

		/**
		 * \brief Convenience constructor from FoV.
		 *
		 * Use this constructor when the calibrated camera pose differs from the true one. The projection will be performed using the true
		 * camera pose while for estimation the calibrated LinearDetector::ProjectionMatrix is used.
		 * 
		 * The sensors center is assumed to be in its middle.
		 *
		 * \param fieldOfView The field of view of the linear detector in degrees.
		 * \param sensorWidth Width of the sensor in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the linear detector with respect to the local frame.
		 *			The detector faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
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
		 * \param sensorWidth Width of the sensor in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the linear detector with respect to the local frame.
		 *			The detector faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
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
		 * \param sensorWidth Width of the sensor in metres.
		 * \param sensorVariance Variance of the zero-mean Gaussian noise that added to true sensor measurements. The value cannot be 0,
		 *			though adding noise can be disabled when projecting a point.
		 * \param pose True pose of the linear detector with respect to the local frame.
		 *			The detector faces in the z-direction, as common in the pinhole camera model. The x-direction is the sensor direction.
		 * \param calibratedPose Calibrated pose of the linear detector with respect to the local frame.
		*/
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);

		// getters
		double focalLength() const; /**< \brief Returns the focal length. */
		double sensorWidth() const; /**< \brief Returns the sensor width. */
		double sensorVariance() const; /**< \brief Returns the sensor variance. */
		double centerOffset() const; /**< \brief Returns the center offset. */
		const Pose3& pose() const; /**< \brief Returns the true pose. */
		const Pose3& calibratedPose() const; /**< \brief Returns the calibrated pose. */
		const ProjectionMatrix& projectionMatrix() const; /**< \brief Returns the true projection matrix. */
		const ProjectionMatrix& calibratedProjectionMatrix() const; /**< \brief Returns the calibrated projection matrix. */

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
		 * If the point lies outside the field of view of the linear detector an OutsideOfFieldOfView exception is thrown.
		 *
		 * \param point The point in the local frame to project.
		 * \param addNoise Add zero-mean Gaussian noise to the true measurement.
		*/
		double safeProjectPoint(const Point3& point, bool addNoise = true);

		// estimation
		/**
		 * \brief Returns the estimation equation corresponding to this linear detector.
		 *
		 * The estimation of a point from linear detector measurements is based on minimising the algebraic 
		 * backprojection error, i.e. the least square solution to the intersecting planes. 
		 *
		 * If the point lies outside the field of view of the linear detector an OutsideOfFieldOfView exception is thrown.
		 *
		 * The estimation equation is of the form:
		 * \f$ \left( \mathbf{p}_{1} - u \, \mathbf{p}_{2} \right) \cdot \mathbf{\hat{x}} = 0 \f$, where \f$ \mathbf{p}_{i} \f$ is the i-th row
		 * of the projection matrix, \f$ u \f$ is the sensor measurement and \f$ \mathbf{\hat{x}} \f$ is the homogeneous position of the point.
		 *
		 * \param point True position of the point.
		 * \param addSensorNoise Add zero-mean Gaussian noise on the true sensor measurement.
		*/
		Eigen::Matrix<double, 1, 4> getEstimationEquation(const Point3& point, bool addSensorNoise = true);
	};

	// custom exception
	/**
	 * \brief Custom exception that is thrown when a point is outside the field of view of the linear detector.
	*/
	class OutsideOfFieldOfView : public std::exception {
		std::string mMsg;

	public:
		/**
		 * \brief Constructor from message.
		 *
		 * \param msg Error message.
		*/
		OutsideOfFieldOfView(const std::string& msg);

		/**
		 * \brief Returns explanatory string.
		*/
		virtual const char* what() const throw ();
	};
}
