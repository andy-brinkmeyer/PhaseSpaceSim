#pragma once

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>

#include <PSS/camera/LinearDetector.h>

#include <boost/optional.hpp>


namespace PSS {
	class __declspec(dllexport) LinearDetectorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
		gtsam::Vector1 mMeasurement;
		LinearDetector::ProjectionMatrix mProjectionMatrix;

	public:
		LinearDetectorFactor(const gtsam::Key key, const double measurement, const LinearDetector::ProjectionMatrix projectionMatrix, const gtsam::SharedNoiseModel& noiseModel);

		gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H) const;
	};
}