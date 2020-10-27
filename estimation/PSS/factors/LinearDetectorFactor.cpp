#include "LinearDetectorFactor.h"

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>

#include <PSS/camera/LinearDetector.h>

#include <Eigen/Core>

#include <cmath>


namespace PSS {
	LinearDetectorFactor::LinearDetectorFactor(const gtsam::Key key, const double measurement, const LinearDetector::ProjectionMatrix projectionMatrix, const gtsam::SharedNoiseModel& noiseModel)
		: gtsam::NoiseModelFactor1<gtsam::Pose3>(noiseModel, key)
		, mMeasurement{ measurement }
		, mProjectionMatrix{ projectionMatrix }
	{ }

	gtsam::Vector LinearDetectorFactor::evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H) const {
		// project the point onto the sensor
		Eigen::Vector2d projectedPoint{ mProjectionMatrix * pose.translation().homogeneous() };
		// compute the jacobian
		if (H) {
			double lSqr{ std::pow(projectedPoint(1), -2) };

			// compute the point jacobian entries
			double j1{ lSqr * ((mProjectionMatrix(0,0) * projectedPoint(1)) - (mProjectionMatrix(1,0) * projectedPoint(0))) };
			double j2{ lSqr * ((mProjectionMatrix(0,1) * projectedPoint(1)) - (mProjectionMatrix(1,1) * projectedPoint(0))) };
			double j3{ lSqr * ((mProjectionMatrix(0,2) * projectedPoint(1)) - (mProjectionMatrix(1,2) * projectedPoint(0))) };

			gtsam::Matrix13 pointJacobian{ j1, j2, j3 };
			pointJacobian = pointJacobian * pose.rotation().matrix();

			// construct the whole jacobian
			*H = (gtsam::Matrix(1,6) << 0, 0, 0, pointJacobian).finished();
		}

		// compute the error
		return (projectedPoint.hnormalized() - mMeasurement);
	}

	LinearDetectorFactor2::LinearDetectorFactor2(const gtsam::Key key, const double measurement, const LinearDetector::ProjectionMatrix projectionMatrix, const gtsam::SharedNoiseModel& noiseModel)
		: gtsam::NoiseModelFactor1<gtsam::Vector3>(noiseModel, key)
		, mMeasurement{ measurement }
		, mProjectionMatrix{ projectionMatrix }
	{ }

	gtsam::Vector LinearDetectorFactor2::evaluateError(const gtsam::Vector3& point, boost::optional<gtsam::Matrix&> H) const {
		// project the point onto the sensor
		Eigen::Vector2d projectedPoint{ mProjectionMatrix * point.homogeneous() };
		// compute the jacobian
		if (H) {
			double lSqr{ std::pow(projectedPoint(1), -2) };

			// compute the point jacobian entries
			double j1{ lSqr * ((mProjectionMatrix(0,0) * projectedPoint(1)) - (mProjectionMatrix(1,0) * projectedPoint(0))) };
			double j2{ lSqr * ((mProjectionMatrix(0,1) * projectedPoint(1)) - (mProjectionMatrix(1,1) * projectedPoint(0))) };
			double j3{ lSqr * ((mProjectionMatrix(0,2) * projectedPoint(1)) - (mProjectionMatrix(1,2) * projectedPoint(0))) };

			gtsam::Matrix13 pointJacobian{ j1, j2, j3 };

			// construct the whole jacobian
			*H = (gtsam::Matrix(1, 3) << pointJacobian).finished();
		}

		// compute the error
		return (projectedPoint.hnormalized() - mMeasurement);
	}
}