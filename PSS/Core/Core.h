#pragma once

#include "../camera/Camera.h"

#include <gtsam/geometry/Point3.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <exception>


namespace PSS {
	class Core {
	public:
		// cosntructors
		Core(const std::unordered_map<std::string, Camera>& cameras);

		// getters
		std::unordered_map<std::string, Camera>& cameras();

		// estimation
		gtsam::Point3 estimateFromCameras(gtsam::Point3& point, std::vector<std::string>& cameras);

	private:
		std::unordered_map<std::string, Camera> mCameras;
	};

	class UnderdeterminedSystem : public std::exception {
	public:
		UnderdeterminedSystem(const std::string& msg);

		virtual const char* what() const throw ();

	private:
		std::string mMsg;
	};
}