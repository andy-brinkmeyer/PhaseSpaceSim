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
		LinearDetector(double fieldOfView, double sensorWidth, gtsam::Pose3& pose);
		LinearDetector(double fieldOfView, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose);
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose);
		LinearDetector(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose, gtsam::Pose3& calibratedPose);

		// getters
		double focalLength();
		double sensorWidth();
		double centerOffset();
		gtsam::Pose3 pose();
		ProjectionMatrix projectionMatrix();
		ProjectionMatrix calibratedProjectionMatrix();

		// projection
		double projectPoint(gtsam::Point3 &point);
		double safeProjectPoint(gtsam::Point3& point);

	private:
		double mFocalLength;
		double mCenterOffset;
		double mSensorWidth;
		gtsam::Pose3 mPose;
		gtsam::Pose3 mCalibratedPose;
		ProjectionMatrix mProjectionMatrix;
		ProjectionMatrix mCalibratedProjectionMatrix;

		// helper functions
		void init(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose, boost::optional<gtsam::Pose3> calibratedPose);
		ProjectionMatrix computeProjectionMatrix(double focalLength, double centerOffset, double sensorWidth, gtsam::Pose3& pose);
	};
}