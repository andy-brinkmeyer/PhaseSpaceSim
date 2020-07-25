#include <PSS/core/SimulationContext.h>
#include <PSS/core/Core.h>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <cmath>


class VelFactor : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
	gtsam::Vector3 mVel;

public:
	VelFactor(gtsam::Key j, const gtsam::Vector3& vel, const gtsam::SharedNoiseModel& model) 
		: gtsam::NoiseModelFactor1<gtsam::Vector3>(model, j), mVel(vel) { }

	gtsam::Vector evaluateError(const gtsam::Vector3& vel, boost::optional<gtsam::Matrix&> H = boost::none) const {
		if (H) (*H) = (gtsam::Matrix(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0).finished();
		return vel - mVel;
	}
};

class BiasFactor : public gtsam::NoiseModelFactor1<gtsam::imuBias::ConstantBias> {
	gtsam::imuBias::ConstantBias mBias;

public:
	BiasFactor(gtsam::Key j, const gtsam::imuBias::ConstantBias& bias, const gtsam::SharedNoiseModel& model)
		: gtsam::NoiseModelFactor1<gtsam::imuBias::ConstantBias>(model, j), mBias(bias) { }

	gtsam::Vector evaluateError(const gtsam::imuBias::ConstantBias& bias, boost::optional<gtsam::Matrix&> H = boost::none) const {
		if (H) (*H) = gtsam::Matrix66::Identity();
		return bias.vector() - mBias.vector();
	}
};

int main(int argc, char* argv[]) {
	if (argc != 4) {
		std::cout << "Invalid number of arguments. Metafile, measurements and output file need to be specified." << std::endl;
		return -1;
	}
	
	// create mocap simulation
	PSS::SimulationContext simContext{ argv[1], argv[2], argv[3] };
	PSS::Core core{ simContext };
	const PSS::Measurement& currentMeasurement{ simContext.currentMeasurement() };

	// skip the first measurements as they are not accurate due to spline fitting
	simContext.nextMeasurement();

	double initFrame{ 1000 };
	while (currentMeasurement.frame < initFrame) {
		simContext.nextMeasurement();
	}

	// define IMU characteristics
	double accelSigma{ 0.0346 };
	double gyroSigma{ 0.0032 };
	gtsam::Matrix33 accelCovariance{ gtsam::I_3x3 * pow(accelSigma, 2) };
	gtsam::Matrix33 gyroCovariance{ gtsam::I_3x3 * pow(gyroSigma, 2) };
	gtsam::Matrix33 integrationCovariance{ gtsam::I_3x3 * 1e-8 };
	gtsam::imuBias::ConstantBias imuBias{ gtsam::Vector3{ 0, 0, 0 }, gtsam::Vector3{ 0, 0, 0 } };

	// define preintegration
	boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams = gtsam::PreintegratedImuMeasurements::Params::MakeSharedD();
	preintegrationParams->accelerometerCovariance = accelCovariance;
	preintegrationParams->gyroscopeCovariance = gyroCovariance;
	preintegrationParams->integrationCovariance = integrationCovariance;

	boost::shared_ptr<gtsam::PreintegratedImuMeasurements> preintegrated = boost::make_shared<gtsam::PreintegratedImuMeasurements>(preintegrationParams, imuBias);

	// define graph
	gtsam::NonlinearFactorGraph* graph = new gtsam::NonlinearFactorGraph();
	gtsam::ISAM2* isam = new gtsam::ISAM2();

	// define used keys
	using gtsam::symbol_shorthand::X; // pose
	using gtsam::symbol_shorthand::V; // velocity
	using gtsam::symbol_shorthand::B; // imu bias

	// define noise
	gtsam::noiseModel::Isotropic::shared_ptr moCapNoise{ gtsam::noiseModel::Isotropic::Sigma(3, 0.001) };
	gtsam::Vector6 poseNoiseVec;
	poseNoiseVec << 0.001, 0.001, 0.001, 0.01, 0.01, 0.01;
	gtsam::noiseModel::Diagonal::shared_ptr poseNoise{ gtsam::noiseModel::Diagonal::Sigmas(poseNoiseVec) };
	gtsam::noiseModel::Isotropic::shared_ptr velocityNoise{ gtsam::noiseModel::Isotropic::Sigma(3, 0.1) };
	gtsam::noiseModel::Isotropic::shared_ptr biasNoise{ gtsam::noiseModel::Isotropic::Sigma(6, 0.1) };

	// define priors
	gtsam::Point3 initPos{ currentMeasurement.position };
	gtsam::Rot3 initRot{ currentMeasurement.rotation.matrix() };
	gtsam::Pose3 initPose{ initRot, initPos };
	gtsam::Vector3 initVel{ currentMeasurement.vel };

	// add priors to graph
	graph->addPrior(X(0), initPose, poseNoise);
	graph->addPrior(V(0), initVel, velocityNoise);
	graph->addPrior(B(0), imuBias, biasNoise);

	// add initial values
	gtsam::Values initValues;
	initValues.insert(X(0), initPose);
	initValues.insert(V(0), initVel);
	initValues.insert(B(0), imuBias);

	// define variables needed in loop
	double prevTime{ currentMeasurement.time };
	double dt;
	size_t frame{ 0 };
	size_t prevFrame{ 0 };
	gtsam::NavState prevState{ initPose, initVel };
	gtsam::imuBias::ConstantBias prevBias{ imuBias };
	gtsam::Values result;
	gtsam::NavState predicted;

	// loop through measurements
	simContext.nextMeasurement();
	while (currentMeasurement.valid) {
		while (currentMeasurement.valid && currentMeasurement.marker != "Marker_1") {
			simContext.nextMeasurement();
		}
		// advance variables
		frame++;
		dt = currentMeasurement.time - prevTime;
		std::cout << "Estimating frame " << frame << std::endl;

		// preintegrate measurements
		preintegrated->integrateMeasurement(-currentMeasurement.accel, currentMeasurement.angVel, dt);

		// estimate point from camera
		gtsam::Point3 moCapEstimate;
		try {
			moCapEstimate = core.estimateFromCameras(currentMeasurement);
		}
		catch (PSS::UnderdeterminedSystem e) {
			prevTime = currentMeasurement.time;
			simContext.nextMeasurement();
			std::cout << "Underdetermined Camera System" << std::endl;
			continue;
		}

		// create IMU factor
		const gtsam::PreintegratedImuMeasurements constPreint{ dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*preintegrated) };
		gtsam::ImuFactor imuFactor{ X(prevFrame), V(prevFrame), X(frame), V(frame), B(prevFrame), constPreint };
		graph->add(imuFactor);

		// create bias factor
		gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> biasFactor{ B(prevFrame), B(frame), imuBias, biasNoise };
		graph->add(biasFactor);

		// create MoCap factor
		gtsam::GPSFactor gpsFactor{ X(frame), moCapEstimate, moCapNoise };
		graph->add(gpsFactor);

		// create vel factor and bias factor
		gtsam::Vector3 vel{ (moCapEstimate - prevState.pose().translation()) / dt };
		VelFactor velFactor{ V(frame), vel, moCapNoise };
		graph->add(velFactor);
		BiasFactor unaryBiasFactor{ B(frame), imuBias, biasNoise };
		graph->add(unaryBiasFactor);

		// predict next state
		gtsam::NavState predicted{ preintegrated->predict(prevState, prevBias) };

		// insert initial values
		initValues.insert(X(frame), predicted.pose());
		initValues.insert(V(frame), predicted.v());
		initValues.insert(B(frame), prevBias);

		// optimize
		try {
			isam->update(*graph, initValues);
			result = isam->calculateEstimate();
		}
		catch (gtsam::IndeterminantLinearSystemException e) {
			std::cout << e.nearbyVariable() << std::endl << "#############" << std::endl;
			std::cout << predicted.pose() << std::endl << "#############" << std::endl;
			std::cout << predicted.v() << std::endl << "#############" << std::endl;
			std::cout << prevBias << std::endl << "#############" << std::endl;
			std::cout << "Acc: " << currentMeasurement.accel << std::endl << "#############" << std::endl;
			std::cout << "Angvel: " << currentMeasurement.angVel << std::endl << "#############" << std::endl;
			std::cout << "#############" << std::endl << isam->marginalCovariance(X(prevFrame)) << std::endl << "#############" << std::endl;
			std::cout << isam->marginalCovariance(V(prevFrame)) << std::endl << "#############" << std::endl;
			std::cout << isam->marginalCovariance(B(prevFrame)) << std::endl << "#############" << std::endl;
			break;
		}

		// save prev values for next iteration
		gtsam::Pose3 estimatedPose{ result.at<gtsam::Pose3>(X(frame)) };
		gtsam::Vector3 estimtedVel{ result.at<gtsam::Vector3>(V(frame)) };
		prevState = gtsam::NavState(estimatedPose, estimtedVel);
		prevBias = result.at<gtsam::imuBias::ConstantBias>(B(frame));

		// reset graph and preintegration
		graph->resize(0);
		initValues.clear();
		preintegrated->resetIntegrationAndSetBias(prevBias);

		// write estimate
		simContext.writeEstimate("Marker_1", estimatedPose.translation(), currentMeasurement);

		// next measurement
		prevTime = currentMeasurement.time;
		prevFrame = frame;
		simContext.nextMeasurement();
	}
	return 1;
}