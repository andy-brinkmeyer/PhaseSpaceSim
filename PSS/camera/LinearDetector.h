#pragma once

#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <random>


namespace PSS {
	class LinearDetector {
	public:
		// typedefs
		typedef Eigen::Matrix<double, 2, 4> ProjectionMatrix;

	private:
		// instrinsics
		double mFocalLength;
		double mSensorWidth;
		double mSensorVariance;
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

		// gaussian noise
		std::random_device mRandomDevice;
		std::mt19937 mRandomGenerator;
		std::normal_distribution<double> mNormalDistribution;

	public:
		// constructor
		LinearDetector(const LinearDetector& linearDetector);
		LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const gtsam::Pose3& pose);
		LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const gtsam::Pose3& pose, const gtsam::Pose3& calibratedPose);
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const gtsam::Pose3& pose);
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const gtsam::Pose3& pose, const gtsam::Pose3& calibratedPose);

		// macro for Eigen to peroperly handle the alignement of fixed size matrices
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		// getters
		double focalLength();
		double sensorWidth();
		double sensorVariance();
		double centerOffset();
		gtsam::Pose3& pose();
		ProjectionMatrix& projectionMatrix();
		ProjectionMatrix& calibratedProjectionMatrix();

		// projection
		double projectPoint(gtsam::Point3 &point, bool addNoise = true);
		double safeProjectPoint(gtsam::Point3& point, bool addNoise = true);

		// estimation
		Eigen::Matrix<double, 1, 4> getEstimationEquation(gtsam::Point3& point, bool addSensorNoise = true);
	};
}