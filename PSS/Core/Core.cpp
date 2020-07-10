#include "Core.h"

#include "../camera/Camera.h"

#include <map>


namespace PSS {
	// constructors
	Core::Core(std::map<string, Camera>& cameras) {
		mCameras = cameras;
	}

	// getters
	std::map<string, Camera>& Core::cameras() { return mCameras; };
}