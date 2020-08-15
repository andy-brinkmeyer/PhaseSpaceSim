#include "Core.h"

#include "../camera/Camera.h"
#include "SimulationContext.h"
#include "../geometry/Pose3.h"

#include <Eigen/Dense>

#include <unordered_map>
#include <vector>
#include <string>
#include <exception>
#include <cstddef>
#include <cmath>


namespace PSS {
	// constructors
	Core::Core(const CameraMap& cameras) : mCameras{ cameras } { }

	Core::Core(SimulationContext& simContext) {
		const std::vector<CameraConfig>& cameraConfigs{ simContext.metaData().cameras };
		for (int i{ 0 }; i < cameraConfigs.size(); i++) {
			Camera camera{ cameraConfigs[i].fieldOfView, cameraConfigs[i].sensorWidth, cameraConfigs[i].resolution, cameraConfigs[i].sensorVariance, cameraConfigs[i].pose, cameraConfigs[i].calibratedPose };
			mCameras.insert({ cameraConfigs[i].id, camera });
		}
	}

	// getters
	const CameraMap& Core::cameras() const { return mCameras; };

	// estimation
	Point3 Core::estimateFromCameras(const Point3& point, const std::vector<std::string>& cameras, bool addSensorNoise) {
		Eigen::Matrix<double, Eigen::Dynamic, 4> estimationEquations;

		for (const std::string& cameraID : cameras) {
			// lookup the camera
			CameraMap::iterator foundCamera{ mCameras.find(cameraID) };
			if (foundCamera == mCameras.end()) {
				continue;
			}
			Camera& currentCamera{ foundCamera->second };

			// get estimation equations and add to estimation matrix
			try {
				Eigen::Matrix<double, Eigen::Dynamic, 4> equations{ currentCamera.getEstimationEquations(point, addSensorNoise) };
				std::ptrdiff_t newEquations{ equations.rows() };
				std::ptrdiff_t oldEquations{ estimationEquations.rows() };
				estimationEquations.conservativeResize(oldEquations + newEquations, Eigen::NoChange);
				for (std::ptrdiff_t i{ 0 }; i < newEquations; i++) {
					estimationEquations.row(oldEquations + i) = equations.row(i);
				}
			} catch (const OutsideOfFieldOfView&) {
				continue;
			}
		}

		// check if enough equations
		if (estimationEquations.rows() < 3) {
			throw UnderdeterminedSystem("Not enough lin. independent equations. Need at least 3, got: " + std::to_string(estimationEquations.rows()));
		}

		// find solution to homogeneous system
		Eigen::BDCSVD<Eigen::MatrixX4d> bdSvd{ estimationEquations.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV) };
		if (bdSvd.rank() < 3) {
			throw UnderdeterminedSystem("System is rank deficient. Rank:" + std::to_string(bdSvd.rank())); 
		}
		else {
			Eigen::Vector4d solution{ bdSvd.matrixV().col(3) };
			Point3 estimatedPoint{ solution.hnormalized() };
			return estimatedPoint;
		}
	}

	Point3 Core::estimateFromCameras(const Measurement& measurement, bool addSensorNoise) {
		return Core::estimateFromCameras(measurement.position, measurement.cameras, addSensorNoise);
	}

	void Core::simulateCameraOnly(SimulationContext& simContext, bool addSensorNoise) {
		// read first measurement
		const Measurement& currentMeasurement{ simContext.currentMeasurement() };
		simContext.nextMeasurement();
		while (currentMeasurement.valid) {
			Point3 estimate;
			try {
				estimate = Point3{ estimateFromCameras(currentMeasurement, addSensorNoise) };
			} catch (const UnderdeterminedSystem e) {
				estimate = Point3{ std::nan(""), std::nan(""), std::nan("") };
			}
			simContext.writeEstimate(currentMeasurement.marker, estimate, currentMeasurement);
			simContext.nextMeasurement();
		}
	}

	// custom exception implementation
	UnderdeterminedSystem::UnderdeterminedSystem(const std::string& msg) : mMsg{ msg } { }

	const char* UnderdeterminedSystem::what() const throw () {
		return mMsg.c_str();
	}
}