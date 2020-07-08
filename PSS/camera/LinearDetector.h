#pragma once

#include <Eigen/Eigen>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>


namespace PSS {
	class LinearDetector {
	public:
		// typedefs
		typedef Eigen::Matrix<double, 2, 4> ProjectionMatrix;

		// constructors
		LinearDetector(double focalLength, double centerOffset, gtsam::Pose3 pose);

		// getters
		double focalLength();
		double centerOffset();
		gtsam::Pose3 pose();
		ProjectionMatrix projectionMatrix();

		// projection
		double projectPoint(gtsam::Point3 point);

	private:
		double mFocalLength;
		double mCenterOffset;
		gtsam::Pose3 mPose;
		ProjectionMatrix mProjectionMatrix;
	};
}