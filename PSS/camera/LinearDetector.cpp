#include "LinearDetector.h"

#include <Eigen/Eigen>
#include <gtsam/geometry/Pose3.h>


namespace PSS {
	// contructors
	LinearDetector::LinearDetector(double focalLength, double centerOffset, gtsam::Pose3 pose) {
		mFocalLength = focalLength;
		mCenterOffset = centerOffset;
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
	double projectPoint() {
		return 1.0;
	}
}
