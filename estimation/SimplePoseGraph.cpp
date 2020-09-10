/**
	* \brief Executable that runs the estimation.
	*
	* The executable takes the following command line arguments:
	*	- estimation type, "camera" or "graph"
	*	- metafile path
	*	- measurements path
	*	- output path for real-time fixed lag smoother
	*	- output path for batch optimisation
	*	- number of frames to skip at the beginning
	*	- smoothing lag
*/

#include <PSS/core/SimulationContext.h>
#include <PSS/core/Core.h>
#include <PSS/camera/Camera.h>
#include <PSS/camera/LinearDetector.h>
#include <PSS/factors/LinearDetectorFactor.h>

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
#include <memory>
#include <fstream>


void poseGraph(PSS::Core& core, PSS::SimulationContext& simContext, const PSS::Measurement& currentMeasurement, double lag, int offset, char* batchFile) {
	// define IMU characteristics
	double accelSigma{ 0.346 };
	double gyroSigma{ 0.032 };
	gtsam::Matrix3 accelCovariance{ gtsam::I_3x3 * pow(accelSigma, 2) };
	gtsam::Matrix3 gyroCovariance{ gtsam::I_3x3 * pow(gyroSigma, 2) };
	double integrationCovFactor{ 0.0033 };
	gtsam::Matrix3 baseIntCov{ gtsam::I_3x3 * 1e-8 };
	gtsam::imuBias::ConstantBias imuBias{ gtsam::Vector3{ 0, 0, 0 }, gtsam::Vector3{ 0, 0, 0 } };

	// define preintegration
	boost::shared_ptr<gtsam::PreintegrationParams> preintegrationParams = gtsam::PreintegratedImuMeasurements::Params::MakeSharedD();
	preintegrationParams->accelerometerCovariance = accelCovariance;
	preintegrationParams->gyroscopeCovariance = gyroCovariance;
	preintegrationParams->integrationCovariance = baseIntCov;

	boost::shared_ptr<gtsam::PreintegratedImuMeasurements> preintegrated = boost::make_shared<gtsam::PreintegratedImuMeasurements>(preintegrationParams, imuBias);

	// define graph
	std::unique_ptr<gtsam::NonlinearFactorGraph> graph = std::make_unique<gtsam::NonlinearFactorGraph>();
	// std::unique_ptr<gtsam::ISAM2> isam = std::make_unique<gtsam::ISAM2>();
	std::unique_ptr<gtsam::IncrementalFixedLagSmoother> isam = std::make_unique<gtsam::IncrementalFixedLagSmoother>(lag);

	std::unique_ptr<gtsam::NonlinearFactorGraph> batchGraph = std::make_unique<gtsam::NonlinearFactorGraph>();

	// define timestamp map
	gtsam::IncrementalFixedLagSmoother::KeyTimestampMap timestamps{ };

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
	gtsam::noiseModel::Isotropic::shared_ptr linearDetectorNoise{ gtsam::noiseModel::Isotropic::Sigma(1, 0.00005) };

	// define priors
	gtsam::Point3 initPos{ currentMeasurement.position };
	gtsam::Rot3 initRot{ currentMeasurement.rotation.matrix() };
	gtsam::Pose3 initPose{ initRot, initPos };
	gtsam::Vector3 initVel{ currentMeasurement.vel };

	// add priors to graph
	graph->addPrior(X(0), initPose, poseNoise);
	graph->addPrior(V(0), initVel, velocityNoise);
	graph->addPrior(B(0), imuBias, biasNoise);
	timestamps[X(0)] = 0.0;
	timestamps[V(0)] = 0.0;
	timestamps[B(0)] = 0.0;
	batchGraph->addPrior(X(0), initPose, poseNoise);
	batchGraph->addPrior(V(0), initVel, velocityNoise);
	batchGraph->addPrior(B(0), imuBias, biasNoise);

	// add initial values
	gtsam::Values initValues{ };
	initValues.insert(X(0), initPose);
	initValues.insert(V(0), initVel);
	initValues.insert(B(0), imuBias);
	std::unique_ptr<gtsam::Values> batchValues = std::make_unique<gtsam::Values>();
	batchValues->insert(X(0), initPose);
	batchValues->insert(V(0), initVel);
	batchValues->insert(B(0), imuBias);

	// define variables needed in loop
	double prevTime{ currentMeasurement.time };
	double dt;
	size_t frame{ 0 };
	size_t prevFrame{ 0 };
	size_t estimatedFrames{ 0 };
	int skippedFrames{ 0 };
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
		std::cout << "Estimating frame " << frame << "\n";

		// preintegrate measurements
		// double normAccel{ currentMeasurement.accel.norm() };
		// preintegrationParams->integrationCovariance = (gtsam::I_3x3 * integrationCovFactor * normAccel) + baseIntCov;
		preintegrated->integrateMeasurement(-currentMeasurement.accel, currentMeasurement.accel, dt);

		// estimate point from camera
		/*gtsam::Point3 moCapEstimate;
		bool haveCameraEstimate;
		try {
			moCapEstimate = core.estimateFromCameras(currentMeasurement);
			haveCameraEstimate = true;
		}
		catch (PSS::UnderdeterminedSystem e) {
			std::cout << "Underdetermined Camera System" << "\n";
			haveCameraEstimate = false;
		}*/

		// create LinearDetectorFactors
		bool haveFactor{ false };
		for (const std::string& cameraID : currentMeasurement.cameras) {
			PSS::CameraMap::iterator foundCamera{ core.cameras().find(cameraID) };
			if (foundCamera == core.cameras().end()) {
				continue;
			}
			PSS::Camera& camera{ foundCamera->second };
			try {
				double measurement{ camera.horizontalDetector().safeProjectPoint(currentMeasurement.position) };
				PSS::LinearDetectorFactor linDetFactor{ X(frame), measurement, camera.horizontalDetector().calibratedProjectionMatrix(), linearDetectorNoise };
				graph->add(linDetFactor);
				batchGraph->add(linDetFactor);
				haveFactor = true;
			}
			catch (PSS::OutsideOfFieldOfView&) { }
			try {
				double measurement{ camera.verticalDetector().safeProjectPoint(currentMeasurement.position) };
				PSS::LinearDetectorFactor linDetFactor{ X(frame), measurement, camera.verticalDetector().calibratedProjectionMatrix(), linearDetectorNoise };
				graph->add(linDetFactor);
				batchGraph->add(linDetFactor);
				haveFactor = true;
			}
			catch (PSS::OutsideOfFieldOfView&) {}
		}

		if (haveFactor or skippedFrames > 3) {
			// create IMU factor
			const gtsam::PreintegratedImuMeasurements constPreint{ dynamic_cast<const gtsam::PreintegratedImuMeasurements&>(*preintegrated) };
			gtsam::ImuFactor imuFactor{ X(prevFrame), V(prevFrame), X(frame), V(frame), B(prevFrame), constPreint };
			graph->add(imuFactor);
			batchGraph->add(imuFactor);

			// create bias factor
			gtsam::BetweenFactor<gtsam::imuBias::ConstantBias> biasFactor{ B(prevFrame), B(frame), imuBias, biasNoise };
			graph->add(biasFactor);
			batchGraph->add(biasFactor);

			// create MoCap factor
			/*gtsam::GPSFactor gpsFactor{ X(frame), moCapEstimate, moCapNoise };
			graph->add(gpsFactor);*/

			// predict next state
			gtsam::NavState predicted{ preintegrated->predict(prevState, prevBias) };

			// insert initial values
			initValues.insert(X(frame), predicted.pose());
			initValues.insert(V(frame), predicted.v());
			initValues.insert(B(frame), prevBias);
			estimatedFrames++;
			timestamps[X(frame)] = estimatedFrames;
			timestamps[V(frame)] = estimatedFrames;
			timestamps[B(frame)] = estimatedFrames;

			batchValues->insert(X(frame), predicted.pose());
			batchValues->insert(V(frame), predicted.v());
			batchValues->insert(B(frame), prevBias);

			// optimize
			isam->update(*graph, initValues, timestamps);
			isam->update();
			result = isam->calculateEstimate();

			// save prev values for next iteration
			gtsam::Pose3 estimatedPose{ result.at<gtsam::Pose3>(X(frame)) };
			gtsam::Vector3 estimtedVel{ result.at<gtsam::Vector3>(V(frame)) };
			prevState = gtsam::NavState(estimatedPose, estimtedVel);
			prevBias = result.at<gtsam::imuBias::ConstantBias>(B(frame));

			// reset graph and preintegration
			graph->resize(0);
			initValues.clear();
			preintegrated->resetIntegrationAndSetBias(prevBias);
			timestamps.clear();

			// write estimate
			simContext.writeEstimate("Marker_1", estimatedPose.translation(), currentMeasurement);

			// advance frame
			prevFrame = frame;
			skippedFrames = 0;
		}
		else skippedFrames++;

		// next measurement
		prevTime = currentMeasurement.time;
		simContext.nextMeasurement();
	}

	std::cout << "Starting batch optimization.\n";
	gtsam::LevenbergMarquardtOptimizer optimizer{ *batchGraph, *batchValues };
	gtsam::Values batchResult = optimizer.optimize();

	std::ofstream output{ batchFile };
	output << "frame,x,y,z\n";
	for (size_t i{ 1 }; i <= frame; i++) {
		if (batchResult.exists(X(i))) {
			gtsam::Pose3 resultPose = batchResult.at<gtsam::Pose3>(X(i));
			output << i+offset << ',' << resultPose.x() << ',' << resultPose.y() << ',' << resultPose.z() << '\n';
		}
	}
}

void cameraEstimation(PSS::Core& core, PSS::SimulationContext& simContext) {
	core.simulateCameraOnly(simContext);
}

int main(int argc, char* argv[]) {
	if (argc < 6) {
		std::cout << "Invalid number of arguments. Estimation type (graph or camera), metafile, measurements, incremental output file, batch output file, frame offset and smooting lag need to be specified.\n";
		return 0;
	}
	
	// create mocap simulation
	PSS::SimulationContext simContext{ argv[2], argv[3], argv[4] };
	PSS::Core core{ simContext };
	const PSS::Measurement& currentMeasurement{ simContext.currentMeasurement() };

	// skip the first measurements since they can be weird due to smoothing spline fitting
	double initFrame;
	if (argc >= 7) {
		initFrame = atoi(argv[6]);
	} else {
		initFrame = 0;
	}
	simContext.nextMeasurement();
	while (currentMeasurement.frame < initFrame) {
		simContext.nextMeasurement();
	}

	// set smoother lag
	double lag;
	if (argc >= 8) {
		lag = atof(argv[7]);
	}
	else {
		lag = 10.0;
	}

	// call appropriate estimation method
	std::string estimationType{ argv[1] };
	if (estimationType == "graph") {
		poseGraph(core, simContext, currentMeasurement, lag, initFrame, argv[5]);
	} else if (estimationType == "camera") {
		cameraEstimation(core, simContext);
	}
	else {
		std::cout << "Estimation type " << argv[1] << " not available. Choose either graph or camera.\n";
	}

	std::cout << "Done!\n";

	return 0;
}