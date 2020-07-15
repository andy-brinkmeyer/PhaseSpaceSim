#include "LinearDetector.h"

#include <stdexcept>
#include <cmath>

#include <boost/none.hpp>
#include <boost/optional.hpp>
#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>


namespace PSS {
	// contructors
	LinearDetector::LinearDetector(const LinearDetector& linearDetector)
		: LinearDetector::LinearDetector(
			linearDetector.mFocalLength,
			linearDetector.mCenterOffset,
			linearDetector.mSensorWidth,
			linearDetector.mSensorVariance,
			linearDetector.mPose,
			linearDetector.mCalibratedPose
		) 
	{ }

	LinearDetector::LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const gtsam::Pose3& pose)
		: LinearDetector::LinearDetector(fieldOfView, sensorWidth, sensorVariance, pose, pose)
	{ }

	LinearDetector::LinearDetector(double fieldOfView, double sensorWidth, double sensorVariance, const gtsam::Pose3& pose, const gtsam::Pose3& calibratedPose)
		: LinearDetector::LinearDetector(sensorWidth / (2 * std::tan(0.5 * (fieldOfView * M_PI / 180))), 0.5 * sensorWidth, sensorWidth, sensorVariance, pose, calibratedPose)
	{ }

	LinearDetector::LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const gtsam::Pose3& pose)
		: LinearDetector::LinearDetector(focalLength, centerOffset, sensorWidth, sensorVariance, pose, pose)
	{ }

	LinearDetector::LinearDetector(double focalLength, double centerOffset, double sensorWidth, double sensorVariance, const gtsam::Pose3& pose, const gtsam::Pose3& calibratedPose)
		: mFocalLength{ focalLength }
		, mCenterOffset{ centerOffset }
		, mSensorVariance{ sensorVariance }
		, mSensorWidth{ sensorWidth }
		, mPose{ pose }
		, mCalibratedPose{ calibratedPose }
		, mProjectionMatrix{ computeProjectionMatrix(mFocalLength, mCenterOffset, mSensorWidth, mPose) }
		, mCalibratedProjectionMatrix{ computeProjectionMatrix(mFocalLength, mCenterOffset, mSensorWidth, mCalibratedPose) }
		, mC1{ mCalibratedProjectionMatrix.row(0) }
		, mC2{ mCalibratedProjectionMatrix.row(1) }
		, mRandomDevice{ }
		, mRandomGenerator{ mRandomDevice() }
		, mNormalDistribution{ 0.0, sensorVariance }
	{ }

	// getters
	double LinearDetector::focalLength() { return mFocalLength; }
	double LinearDetector::sensorWidth() { return mSensorWidth; }
	double LinearDetector::sensorVariance() { return mSensorVariance; }
	double LinearDetector::centerOffset() { return mCenterOffset; }
	gtsam::Pose3& LinearDetector::pose() { return mPose; }
	LinearDetector::ProjectionMatrix& LinearDetector::projectionMatrix() { return mProjectionMatrix; }
	LinearDetector::ProjectionMatrix& LinearDetector::calibratedProjectionMatrix() { return mCalibratedProjectionMatrix; }

	// projection
	double LinearDetector::projectPoint(gtsam::Point3 &point, bool addNoise) {
		Eigen::Matrix<double, 4, 1> pointH{ point.homogeneous() };
		Eigen::Matrix<double, 2, 1> projectedH{ mProjectionMatrix * pointH };
		double projected;
		if (addNoise) {
			projected = projectedH.hnormalized()(0, 0) + mNormalDistribution(mRandomGenerator);
		}
		else {
			projected = projectedH.hnormalized()(0, 0);
		}
		return projected;
	}

	double LinearDetector::safeProjectPoint(gtsam::Point3 &point, bool addNoise) {
		// transform the point to camera frame
		gtsam::Point3 pointCamera{ mPose.transformTo(point) };
		
		// check if it is in front of the camera
		if (pointCamera.z() < 0) {
			throw std::domain_error("Point lies behind the linear detector.");
		}

		// check if the point is projected on the sensor
		double projectedPoint{ projectPoint(point, addNoise) };
		if (projectedPoint < 0 || projectedPoint > mSensorWidth) {
			throw std::domain_error("Point lies outside of sensor range.");
		}

		return projectedPoint;
	}

	LinearDetector::ProjectionMatrix LinearDetector::computeProjectionMatrix(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose) {
		Eigen::Matrix<double, 2, 3> intrinsics;
		intrinsics << focalLength, 0, centerOffset,
			0, 0, 1;

		Eigen::Matrix<double, 3, 3> rotation = pose.rotation().matrix();

		Eigen::Matrix<double, 3, 4> translation;
		translation << 1, 0, 0, -pose.x(),
			0, 1, 0, -pose.y(),
			0, 0, 1, -pose.z();

		// finally create the projection matrix
		return intrinsics * rotation * translation;
	}

	// estimation
	Eigen::Matrix<double, 1, 4> LinearDetector::getEstimationEquation(gtsam::Point3& point, bool addSensorNoise) {
		double measurement{ safeProjectPoint(point, addSensorNoise) };
		Eigen::Matrix<double, 1, 4> estimationEquation{ (measurement * mC2) - mC1 };
		return estimationEquation;
	}
}
