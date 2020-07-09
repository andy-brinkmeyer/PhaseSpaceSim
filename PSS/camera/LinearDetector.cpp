#include "LinearDetector.h"

#include <stdexcept>

#include <Eigen/Eigen>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>


namespace PSS {
	// contructors
	LinearDetector::LinearDetector(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3 &pose) {
		mFocalLength = focalLength;
		mCenterOffset = centerOffset;
		mSensorWidth = sensorWidth;
		mPose = pose;

		// create the individual camera matrices
		Eigen::Matrix<double, 2, 3> intrinsics;
		intrinsics <<	mFocalLength, 0, mCenterOffset, 
						0, 0, 1;

		Eigen::Matrix<double, 3, 3> rotation = mPose.rotation().matrix();

		Eigen::Matrix<double, 3, 4> translation;
		translation <<	1, 0, 0, -mPose.x(),
						0, 1, 0, -mPose.y(),
						0, 0, 1, -mPose.z();

		// finally create the projection matrix
		mProjectionMatrix = intrinsics * rotation * translation;
	}

	// getters
	double LinearDetector::focalLength() { return mFocalLength; }
	double LinearDetector::centerOffset() { return mCenterOffset; }
	gtsam::Pose3 LinearDetector::pose() { return mPose; }
	LinearDetector::ProjectionMatrix LinearDetector::projectionMatrix() { return mProjectionMatrix; }

	// projection
	double LinearDetector::projectPoint(gtsam::Point3 &point) {
		Eigen::Matrix<double, 4, 1> pointH{ point.homogeneous() };
		Eigen::Matrix<double, 2, 1> projectedH{ mProjectionMatrix * pointH };
		return projectedH.hnormalized()(0,0);
	}

	double LinearDetector::safeProjectPoint(gtsam::Point3 &point) {
		// transform the point to camera frame
		gtsam::Point3 pointCamera{ mPose.transformTo(point) };
		
		// check if it is in front of the camera
		if (pointCamera.z() < 0) {
			throw domain_error("Point lies behind the linear detector.");
		}

		return projectPoint(point);
	}
}
