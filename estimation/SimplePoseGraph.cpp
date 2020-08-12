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
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <cmath>
#include <string>


void poseGraph(PSS::Core& core, PSS::SimulationContext& simContext, const PSS::Measurement& currentMeasurement) {
	// define IMU characteristics
	double accelSigma{ 0.346 };
	double gyroSigma{ 0.032 };
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
	gtsam::IncrementalFixedLagSmoother* isam = new gtsam::IncrementalFixedLagSmoother(1.0);

	// define used keys
	using gtsam::symbol_shorthand::X; // pose
	using gtsam::symbol_shorthand::V; // velocity
	using gtsam::symbol_shorthand::B; // imu bias

	// define noise
	gtsam::noiseModel::Isotropic::shared_ptr moCapNoise{ gtsam::noiseModel::Isotropic::Sigma(3, 0.01) };
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
		bool haveCameraEstimate;
		try {
			moCapEstimate = core.estimateFromCameras(currentMeasurement);
			haveCameraEstimate = true;
		}
		catch (PSS::UnderdeterminedSystem e) {
			std::cout << "Underdetermined Camera System" << std::endl;
			haveCameraEstimate = false;
		}

		gtsam::GPSFactor gpsFactor;
		if (haveCameraEstimate) {
			// create IMU factor
			const gtsam::PreintegratedImuMeasurements constPreint{ dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*preintegrated) };
			gtsam::ImuFactor imuFactor{ X(prevFrame), V(prevFrame), X(frame), V(frame), B(prevFrame), constPreint };
			graph->add(imuFactor);

			// create bias factor
			gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> biasFactor{ B(prevFrame), B(frame), imuBias, biasNoise };
			graph->add(biasFactor);

			// create MoCap factor
			gpsFactor = gtsam::GPSFactor{ X(frame), moCapEstimate, moCapNoise };
			graph->add(gpsFactor);

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
			catch (std::exception& e) {
				std::cout << e.what() << std::endl << "#############" << std::endl;
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
			catch (...) {
				std::cout << "Exception" << std::endl;
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

			// advance frame
			prevFrame = frame;
		}

		// next measurement
		prevTime = currentMeasurement.time;
		simContext.nextMeasurement();
	}
}

void cameraEstimation(PSS::Core& core, PSS::SimulationContext& simContext) {
	core.simulateCameraOnly(simContext);
}

int main(int argc, char* argv[]) {
	if (argc != 5) {
		std::cout << "Invalid number of arguments. Estimation type (graph or camera), metafile, measurements and output file need to be specified." << std::endl;
		return -1;
	}
	
	// create mocap simulation
	PSS::SimulationContext simContext{ argv[2], argv[3], argv[4] };
	PSS::Core core{ simContext };
	const PSS::Measurement& currentMeasurement{ simContext.currentMeasurement() };

	// skip the first measurements as they are not accurate due to spline fitting
	simContext.nextMeasurement();

	double initFrame{ 1000 };
	while (currentMeasurement.frame < initFrame) {
		simContext.nextMeasurement();
	}

	std::string estimationType{ argv[1] };
	if (estimationType == "graph") {
		poseGraph(core, simContext, currentMeasurement);
	} else if (estimationType == "camera") {
		cameraEstimation(core, simContext);
	}
	else {
		std::cout << "Estimation type " << argv[1] << " not available. Choose either graph or camera." << std::endl;
	}

	std::cout << "Done!";

	return 1;
}