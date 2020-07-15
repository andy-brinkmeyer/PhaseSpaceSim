#include "SimulationContext.h"

#include <json/json.hpp>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>

#include <fstream>
#include <string>
#include <vector>


namespace PSS {
	// constructors
	SimulationContext::SimulationContext(const std::string& metaPath, const std::string& measurementsPath, const std::string& outputPath)
	: mMetaPath{ metaPath }
	, mMeasuremtsPath{ measurementsPath }
	, mOutputPath{ outputPath }
	{
		// parse the json
		std::ifstream metaStream{ mMetaPath };
		metaStream >> mJson;

		// map json to structs
		std::vector<CameraConfig> cameras;
		for (int i{ 0 }; i < mJson["cameras"].size(); i++) {
			nlohmann::json cameraObject{ mJson["cameras"][i] };
			gtsam::Point3 cameraPosition{ 
				cameraObject["position"]["x"],
				cameraObject["position"]["y"],
				cameraObject["position"]["z"]
			};
			gtsam::Rot3 cameraRotation{
				cameraObject["rotation"]["q0"],
				cameraObject["rotation"]["q1"],
				cameraObject["rotation"]["q2"],
				cameraObject["rotation"]["q3"]
			};
			gtsam::Pose3 cameraPose{ cameraRotation, cameraPosition };
			CameraConfig cameraConfig{
				cameraObject["id"],
				cameraPose
			};
			cameras.push_back(cameraConfig);
		}
		mMetaData.cameras = cameras;

		std::vector<std::string> markers;
		for (int i{ 0 }; i < mJson["markers"].size(); i++) {
			markers.push_back(mJson["markers"][i]);
		}
		mMetaData.markers = markers;

		mMetaData.samplingRate = mJson["samplingRate"];
	}

	// getters
	MetaData& SimulationContext::metaData() { return mMetaData; }
}