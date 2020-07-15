#pragma once

#include <json/json.hpp>
#include <gtsam/geometry/Pose3.h>

#include <string>
#include <vector>


namespace PSS {
	// structs that resemble the metadata
	struct CameraConfig {
		std::string id;
		gtsam::Pose3 pose;
	};

	struct MetaData {
		std::vector<CameraConfig> cameras;
		std::vector<std::string> markers;
		double samplingRate;
	};

	// simulation context class
	class SimulationContext {
		// file paths
		std::string mMetaPath;
		std::string mMeasuremtsPath;
		std::string mOutputPath;

		// json object
		nlohmann::json mJson;

		// extracted metadata
		MetaData mMetaData;

	public:
		// constructors
		SimulationContext(const std::string& metaPath, const std::string& measurementsPath, const std::string& outputPath);

		// getters
		MetaData& metaData();
	};
}