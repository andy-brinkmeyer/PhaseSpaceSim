#pragma once

#include "../../../PSS_EXPORT.h"

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Key.h>
#include <gtsam/linear/NoiseModel.h>

#include <PSS/camera/LinearDetector.h>

#include <boost/optional.hpp>


namespace PSS {
	/**
	 * \brief Class that represents a raw measurement of a linear detector.
	*/
	class PSS_EXPORT LinearDetectorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
		gtsam::Vector1 mMeasurement;
		LinearDetector::ProjectionMatrix mProjectionMatrix;

	public:
		/**
		 * \brief Standard constructor.
		 *
		 * \param key Key of the variable the factor is acton on.
		 * \param measurement Raw linear detector measurement.
		 * \param projectionMatrix Projection matrix of the linear detector.
		 * \param noiseModel Noise model of the sensor measurement.
		*/
		LinearDetectorFactor(const gtsam::Key key, const double measurement, const LinearDetector::ProjectionMatrix projectionMatrix, const gtsam::SharedNoiseModel& noiseModel);

		gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&> H) const;
	};
}
