#pragma once

#include "../camera/Camera.h"

#include <vector>


namespace PSS {
	class Core {
	public:
		// cosntructors
		Core(std::vector<Camera>& cameras);

		// getters
		std::vector<Camera>& cameras();

	private:
		std::vector<Camera> mCameras;
	};
}