#pragma once

#include "../geometry/Pose3.h"

#include <Eigen/Core>

#include <random>


namespace PSS {
	class LinearDetector {
	public:
		// typedefs
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
		ProjectionMatrix computeProjectionMatrix(double focalLength, double centerOffset, double sensorWidth, Pose3& pose);

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
		LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose);
		LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose);
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose);

		// getters
		double focalLength() const;
		double sensorWidth() const;
		double sensorVariance() const;
		double centerOffset() const;
		const Pose3& pose() const;
		const ProjectionMatrix& projectionMatrix() const;
		const ProjectionMatrix& calibratedProjectionMatrix() const;

		// projection
		double projectPoint(const Point3 &point, bool addNoise = true);
		double safeProjectPoint(const Point3& point, bool addNoise = true);

		// estimation
		Eigen::Matrix<double, 1, 4> getEstimationEquation(const Point3& point, bool addSensorNoise = true);
	};
}
