#pragma once

#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>


namespace PSS {
	class LinearDetector {
	public:
		// typedefs
		typedef Eigen::Matrix<double, 2, 4> ProjectionMatrix;

	private:
		// instrinsics
		double mFocalLength;
		double mSensorWidth;
		double mCenterOffset;

		// pose
		gtsam::Pose3 mPose;
		gtsam::Pose3 mCalibratedPose;

		// projection
		ProjectionMatrix mProjectionMatrix;
		ProjectionMatrix mCalibratedProjectionMatrix;
		ProjectionMatrix computeProjectionMatrix(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose);

		// estimation
		Eigen::Matrix<double, 1, 4> mC1;	// first row of projection matrix for quick access
		Eigen::Matrix<double, 1, 4> mC2;	// second row of projection matrix for quick access
		

	public:
		// macro for Eigen to peroperly handle the alignement of fixed size matrices
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// constructors
		LinearDetector(double fieldOfView, double sensorWidth, const gtsam::Pose3& pose);
		LinearDetector(double fieldOfView, double sensorWidth, const gtsam::Pose3& pose, const gtsam::Pose3& calibratedPose);
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, const gtsam::Pose3& pose);
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, const gtsam::Pose3& pose, const gtsam::Pose3& calibratedPose);

		// getters
		double focalLength();
		double sensorWidth();
		double centerOffset();
		gtsam::Pose3& pose();
		ProjectionMatrix& projectionMatrix();
		ProjectionMatrix& calibratedProjectionMatrix();

		// projection
		double projectPoint(gtsam::Point3 &point);
		double safeProjectPoint(gtsam::Point3& point);

		// estimation
		Eigen::Matrix<double, 1, 4> getEstimationEquation(gtsam::Point3& point);
	};
}