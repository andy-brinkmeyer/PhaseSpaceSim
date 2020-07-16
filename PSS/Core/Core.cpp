#include "Core.h"

#include "../camera/Camera.h"
#include "SimulationContext.h"

#include <Eigen/Dense>
#include <gtsam/geometry/Point3.h>

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
			Camera camera{ cameraConfigs[i].fieldOfView, cameraConfigs[i].sensorWidth, cameraConfigs[i].sensorVariance, cameraConfigs[i].pose };
			mCameras.insert({ cameraConfigs[i].id, camera });
		}
	}

	// getters
	const CameraMap& Core::cameras() const { return mCameras; };

	// estimation
	gtsam::Point3 Core::estimateFromCameras(const gtsam::Point3& point, const std::vector<std::string>& cameras, bool addSensorNoise) {
		Eigen::Matrix<double, Eigen::Dynamic, 4> estimationEquations;

		for (const std::string& cameraID : cameras) {
			// lookup the camera
			CameraMap::const_iterator foundCamera{ mCameras.find(cameraID) };
			if (foundCamera == mCameras.end()) {
				continue;
			}
			Camera currentCamera = foundCamera->second;

			// get estimation equations and add to estimation matrix
			try {
				Eigen::Matrix<double, Eigen::Dynamic, 4> equations{ currentCamera.getEstimationEquations(point, addSensorNoise) };
				std::ptrdiff_t newEquations{ equations.rows() };
				std::ptrdiff_t oldEquations{ estimationEquations.rows() };
				estimationEquations.conservativeResize(oldEquations + newEquations, Eigen::NoChange);
				for (std::ptrdiff_t i{ 0 }; i < newEquations; i++) {
					estimationEquations.row(oldEquations + i) = equations.row(i);
				}
			} catch (const std::domain_error& e) {
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
			gtsam::Point3 estimatedPoint{ solution.hnormalized() };
			return estimatedPoint;
		}
	}

	void Core::simulateCameraOnly(SimulationContext& simContext, bool addSensorNoise) {
		// read first measurement
		const Measurement& currentMeasurement{ simContext.currentMeasurement() };
		simContext.nextMeasurement();
		while (currentMeasurement.valid) {
			gtsam::Point3 estimate;
			try {
				estimate = gtsam::Point3{ estimateFromCameras(currentMeasurement.position, currentMeasurement.cameras, addSensorNoise) };
			} catch (const UnderdeterminedSystem e) {
				estimate = gtsam::Point3{ std::nan(""), std::nan(""), std::nan("") };
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