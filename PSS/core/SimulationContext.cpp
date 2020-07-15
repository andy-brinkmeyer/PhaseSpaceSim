#include "SimulationContext.h"

#include <json/json.hpp>
#include <Eigen/Core>
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
	, mCsvReader{ measurementsPath }
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

		// prepare the csv reader
		mCsvReader.read_header(io::ignore_no_column, 
			"frame", "t", "markerID", "cameras", "x", "y", "z", "q0", "q1", "q2", "q3", "ax", "ay", "az", "wx", "wy", "wz");
	}

	// getters
	MetaData& SimulationContext::metaData() { return mMetaData; }

	// read measurements
	Measurement& SimulationContext::nextMeasurement() {
		// define temp variables to hold values for row parsing
		std::string camerasString;
		double x, y, z;
		double q0, q1, q2, q3;
		double ax, ay, az;
		double wx, wy, wz;

		// read the line
		mCurrentMeasurement.valid = mCsvReader.read_row(
			mCurrentMeasurement.frame,
			mCurrentMeasurement.time,
			mCurrentMeasurement.marker,
			camerasString,
			x, y, z,
			q0, q1, q2, q3,
			ax, ay, az,
			wx, wy, wz
		);
		if (mCurrentMeasurement.valid) {
			// create the cameras vector
			std::vector<std::string> cameras;
			char delimiter{ ';' };
			size_t pos{ 0 };
			std::string token;
			while ((pos = camerasString.find(delimiter)) != std::string::npos) {
				token = camerasString.substr(0, pos);
				cameras.push_back(token);
				camerasString.erase(0, pos + 1);
			}
			cameras.push_back(camerasString);

			// create all the other data types
			mCurrentMeasurement.position = gtsam::Point3{ x, y, z };
			mCurrentMeasurement.rotation = gtsam::Rot3{ q0, q1, q2, q3 };
			mCurrentMeasurement.accel = Eigen::Vector3d{ ax, ay, az };
			mCurrentMeasurement.angVel = Eigen::Vector3d{ wx, wy, wz };
		}

		return mCurrentMeasurement;
	}
}