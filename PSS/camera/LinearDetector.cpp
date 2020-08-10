#define _USE_MATH_DEFINES

#include "LinearDetector.h"
#include "../geometry/Pose3.h"

#include <Eigen/Dense>

#include <stdexcept>
#include <cmath>


namespace PSS {
	// contructors
	LinearDetector::LinearDetector(const LinearDetector& linearDetector)
		: LinearDetector::LinearDetector(
			linearDetector.mFocalLength,
			linearDetector.mCenterOffset,
			linearDetector.mSensorWidth,
			linearDetector.mResolution,
			linearDetector.mSensorVariance,
			linearDetector.mPose,
			linearDetector.mCalibratedPose
		)
	{ }

	LinearDetector::LinearDetector(double fieldOfView, double sensorWidth, int resolution, double sensorVariance, const Pose3& pose)
		: LinearDetector::LinearDetector(fieldOfView, sensorWidth, resolution, sensorVariance, pose, pose)
	{ }

	LinearDetector::LinearDetector(double fieldOfView, double sensorWidth, int resolution, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose)
		: LinearDetector::LinearDetector(sensorWidth / (2 * std::tan(0.5 * (fieldOfView * M_PI / 180))), 0.5 * sensorWidth, sensorWidth, resolution, sensorVariance, pose, calibratedPose)
	{ }

	LinearDetector::LinearDetector(double focalLength, double centerOffset, double sensorWidth, int resolution, double sensorVariance, const Pose3& pose)
		: LinearDetector::LinearDetector(focalLength, centerOffset, sensorWidth, resolution, sensorVariance, pose, pose)
	{ }

	LinearDetector::LinearDetector(double focalLength, double centerOffset, double sensorWidth, int resolution, double sensorVariance, const Pose3& pose, const Pose3& calibratedPose)
		: mFocalLength{ focalLength }
		, mCenterOffset{ centerOffset }
		, mSensorVariance{ sensorVariance }
		, mSensorWidth{ sensorWidth }
		, mResolution{ resolution }
		, mPixelSize{ sensorWidth / resolution }
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
	double LinearDetector::focalLength() const { return mFocalLength; }
	double LinearDetector::sensorWidth() const { return mSensorWidth; }
	int LinearDetector::resolution() const { return mResolution; }
	double LinearDetector::sensorVariance() const { return mSensorVariance; }
	double LinearDetector::centerOffset() const { return mCenterOffset; }
	const Pose3& LinearDetector::pose() const { return mPose; }
	const Pose3& LinearDetector::calibratedPose() const { return mCalibratedPose; }
	const LinearDetector::ProjectionMatrix& LinearDetector::projectionMatrix() const { return mProjectionMatrix; }
	const LinearDetector::ProjectionMatrix& LinearDetector::calibratedProjectionMatrix() const { return mCalibratedProjectionMatrix; }

	// projection
	double LinearDetector::projectPoint(const Point3 &point, bool addNoise) {
		Eigen::Vector4d pointH{ point.homogeneous() };
		Eigen::Vector2d projectedH{ mProjectionMatrix * pointH };
		double projected;
		if (addNoise) {
			projected = projectedH.hnormalized()(0, 0) + mNormalDistribution(mRandomGenerator);
		}
		else {
			projected = projectedH.hnormalized()(0, 0);
		}

		// quantizise the measurement
		if (mResolution > 0) {
			double pixel{ std::round(projected / mPixelSize) }; // round to nearest pixel
			return pixel * mPixelSize;
		}
		else {
			return projected;
		}
	}

	double LinearDetector::safeProjectPoint(const Point3 &point, bool addNoise) {
		// transform the point to camera frame
		Point3 pointCamera{ mPose.transformTo(point) };

		// check if it is in front of the camera
		if (pointCamera.z() < 0) {
			throw OutsideOfFieldOfView("Point lies behind the linear detector.");
		}

		// check if the point is projected on the sensor
		double projectedPoint{ projectPoint(point, addNoise) };
		if (projectedPoint < 0 || projectedPoint > mSensorWidth) {
			throw OutsideOfFieldOfView("Point lies outside of sensor range.");
		}

		return projectedPoint;
	}

	LinearDetector::ProjectionMatrix LinearDetector::computeProjectionMatrix(double focalLength, double centerOffset, double sensorWidth, Pose3& pose) {
		Eigen::Matrix<double, 2, 3> intrinsics;
		intrinsics << focalLength, 0, centerOffset,
			0, 0, 1;

		Eigen::Matrix<double, 3, 3> rotation = pose.rotation().matrix().transpose();

		Eigen::Matrix<double, 3, 4> translation;
		translation << 1, 0, 0, -pose.x(),
			0, 1, 0, -pose.y(),
			0, 0, 1, -pose.z();

		// finally create the projection matrix
		return intrinsics * rotation * translation;
	}

	// estimation
	Eigen::Matrix<double, 1, 4> LinearDetector::getEstimationEquation(const Point3& point, bool addSensorNoise) {
		double measurement{ safeProjectPoint(point, addSensorNoise) };
		Eigen::Matrix<double, 1, 4> estimationEquation{ (measurement * mC2) - mC1 };
		return estimationEquation;
	}

	// custom exception implementation
	OutsideOfFieldOfView::OutsideOfFieldOfView(const std::string& msg) : mMsg{ msg } { }

	const char* OutsideOfFieldOfView::what() const throw () {
		return mMsg.c_str();
	}
}
