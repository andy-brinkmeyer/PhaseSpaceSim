#pragma once

#include "../camera/Camera.h"
#include "SimulationContext.h"
#include "../geometry/Pose3.h"

#include <unordered_map>
#include <vector>
#include <string>
#include <exception>


namespace PSS {
	typedef std::unordered_map<std::string, Camera> CameraMap;

	class Core {
		CameraMap mCameras;

	public:
		// cosntructors
		Core(const CameraMap& cameras);
		Core(SimulationContext& simContext);

		// getters
		const CameraMap& cameras() const;

		// estimation
		Point3 estimateFromCameras(const Point3& point, const std::vector<std::string>& cameras, bool addSensorNoise = true);
		void simulateCameraOnly(SimulationContext& simContext, bool addSensorNoise = true);
	};

	// custom exception
	class UnderdeterminedSystem : public std::exception {
		std::string mMsg;

	public:
		UnderdeterminedSystem(const std::string& msg);

		virtual const char* what() const throw ();
	};
}