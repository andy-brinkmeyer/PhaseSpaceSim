#pragma once

#include "../camera/Camera.h"

#include <gtsam/geometry/Point3.h>

#include <map>


namespace PSS {
	class Core {
	public:
		// cosntructors
		Core(std::map<string, Camera>& cameras);

		// getters
		std::map<string, Camera>& cameras();

		// estimation
		gtsam::Point3& estimateFromPoint(gtsam::Point3& point);

	private:
		std::map<string, Camera> mCameras;
	};
}