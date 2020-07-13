#pragma once

#include "../camera/Camera.h"

#include <gtsam/geometry/Point3.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <exception>


namespace PSS {
	typedef std::unordered_map<std::string, Camera, std::hash<std::string>, std::equal_to<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Camera>>> CameraMap;

	class Core {
		CameraMap mCameras;

	public:
		// cosntructors
		Core(const CameraMap& cameras);

		// getters
		CameraMap& cameras();

		// estimation
		gtsam::Point3 estimateFromCameras(gtsam::Point3& point, std::vector<std::string>& cameras);
	};

	class UnderdeterminedSystem : public std::exception {
		std::string mMsg;

	public:
		UnderdeterminedSystem(const std::string& msg);

		virtual const char* what() const throw ();
	};
}