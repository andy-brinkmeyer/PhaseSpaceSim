#include "LinearDetector.h"

#include <stdexcept>
#include <cmath>

#include <boost/none.hpp>
#include <boost/optional.hpp>
#include <Eigen/Eigen>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>


namespace PSS {
	// contructors
	LinearDetector::LinearDetector(double fieldOfView, double sensorWidth, gtsam::Pose3& pose) {
		double fovRad{ fieldOfView * M_PI / 180 };
		double focalLength{ sensorWidth / (2 * std::tan(0.5 * fovRad)) };
		double centerOffset = 0.5 * sensorWidth;
		boost::optional<gtsam::Pose3> optionalCalibratedPose;
		init(focalLength, centerOffset, sensorWidth, pose, optionalCalibratedPose);
	}

	LinearDetector::LinearDetector(double fieldOfView, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose) {
		double fovRad{ fieldOfView * M_PI / 180 };
		double focalLength{ sensorWidth / (2 * std::tan(0.5 * fovRad)) };
		double centerOffset = 0.5 * sensorWidth;
		boost::optional<gtsam::Pose3> optionalCalibratedPose{ calibratedPose };
		init(focalLength, centerOffset, sensorWidth, pose, optionalCalibratedPose);
	}

	LinearDetector::LinearDetector(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose) {
		boost::optional<gtsam::Pose3> optionalCalibratedPose;
		init(focalLength, centerOffset, sensorWidth, pose, optionalCalibratedPose);
	}

	LinearDetector::LinearDetector(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose) {
		boost::optional<gtsam::Pose3> optionalCalibratedPose{ calibratedPose };
		init(focalLength, centerOffset, sensorWidth, pose, optionalCalibratedPose);
	}

	// getters
	double LinearDetector::focalLength() { return mFocalLength; }
	double LinearDetector::sensorWidth() { return mSensorWidth; }
	double LinearDetector::centerOffset() { return mCenterOffset; }
	gtsam::Pose3& LinearDetector::pose() { return mPose; }
	LinearDetector::ProjectionMatrix& LinearDetector::projectionMatrix() { return mProjectionMatrix; }
	LinearDetector::ProjectionMatrix& LinearDetector::calibratedProjectionMatrix() { return mCalibratedProjectionMatrix; }

	// projection
	double LinearDetector::projectPoint(gtsam::Point3 &point) {
		Eigen::Matrix<double, 4, 1> pointH{ point.homogeneous() };
		Eigen::Matrix<double, 2, 1> projectedH{ mProjectionMatrix * pointH };
		return projectedH.hnormalized()(0, 0);
	}

	double LinearDetector::safeProjectPoint(gtsam::Point3 &point) {
		// transform the point to camera frame
		gtsam::Point3 pointCamera{ mPose.transformTo(point) };
		
		// check if it is in front of the camera
		if (pointCamera.z() < 0) {
			throw std::domain_error("Point lies behind the linear detector.");
		}

		// check if the point is projected on the sensor
		double projectedPoint{ projectPoint(point) };
		if (projectedPoint < 0 || projectedPoint + mCenterOffset > mSensorWidth) {
			throw std::domain_error("Point lies outside of sensor range.");
		}

		return projectedPoint;
	}

	// helper functions
	void LinearDetector::init(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose, boost::optional<gtsam::Pose3> optionalCalibratedPose) {
		mFocalLength = focalLength;
		mCenterOffset = centerOffset;
		mSensorWidth = sensorWidth;
		mPose = pose;

		// create the projection matrices
		mProjectionMatrix = computeProjectionMatrix(mFocalLength, mCenterOffset, mSensorWidth, mPose);
		if (optionalCalibratedPose) {
			mCalibratedPose = *optionalCalibratedPose;
			mCalibratedProjectionMatrix = computeProjectionMatrix(mFocalLength, mCenterOffset, mSensorWidth, mCalibratedPose);
		}
		else {
			mCalibratedPose = mPose;
			mCalibratedProjectionMatrix = mProjectionMatrix;
		}
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
}
