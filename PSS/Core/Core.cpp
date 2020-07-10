#include "Core.h"

#include "../camera/Camera.h"

#include <vector>


namespace PSS {
	// constructors
	Core::Core(std::vector<Camera>& cameras) {
		mCameras = cameras;
	}

	// getters
	std::vector<Camera>& Core::cameras() { return mCameras; };
}