#include "SimulationContext.h"

#include "../geometry/Rot3.h"
#include "../geometry/Pose3.h"

#include <json/json.hpp>
#include <Eigen/Core>

#include <fstream>
#include <string>
#include <vector>
#include <sstream>


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
			Point3 cameraPosition{ 
				cameraObject["position"]["x"],
				cameraObject["position"]["y"],
				cameraObject["position"]["z"]
			};
			Rot3 cameraRotation{
				cameraObject["rotation"]["q0"],
				cameraObject["rotation"]["q1"],
				cameraObject["rotation"]["q2"],
				cameraObject["rotation"]["q3"]
			};
			Pose3 cameraPose{ cameraRotation, cameraPosition };

			// check if calibrated position and rotation are provided
			Point3 calibratedCameraPosition;
			if (cameraObject["calibratedPosition"] == nullptr) {
				calibratedCameraPosition = Point3{ cameraPosition };
			}
			else {
				calibratedCameraPosition = Point3{
				cameraObject["calibratedPosition"]["x"],
				cameraObject["calibratedPosition"]["y"],
				cameraObject["calibratedPosition"]["z"]
				};
			}
			Rot3 calibratedCameraRotation;
			if (cameraObject["calibratedRotation"] == nullptr) {
				calibratedCameraRotation = Rot3{ cameraRotation };
			}
			else {
				calibratedCameraRotation = Rot3{
				cameraObject["calibratedRotation"]["q0"],
				cameraObject["calibratedRotation"]["q1"],
				cameraObject["calibratedRotation"]["q2"],
				cameraObject["calibratedRotation"]["q3"]
				};
			}
			Pose3 calibratedCameraPose{ calibratedCameraRotation, calibratedCameraPosition };
			CameraConfig cameraConfig{
				cameraObject["id"],
				cameraPose,
				calibratedCameraPose,
				cameraObject["fieldOfView"],
				cameraObject["sensorWidth"],
				cameraObject["sensorVariance"]
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
			"frame", "t", "markerID", "cameras", "x", "y", "z", "q0", "q1", "q2", "q3", "ax", "ay", "az", "wx", "wy", "wz", "vx", "vy", "vz");

		// prepare the output
		mOutputStream.open(mOutputPath);
		std::string header{ "frame,t,marker,trueX,trueY,trueZ,x,y,z" };
		mOutputStream << header << std::endl;
	}

	// destructor
	SimulationContext::~SimulationContext() {
		mOutputStream.close();
	}

	// getters
	const MetaData& SimulationContext::metaData() { return mMetaData; }
	const Measurement& SimulationContext::currentMeasurement() { return mCurrentMeasurement; }

	// read measurements
	Measurement& SimulationContext::nextMeasurement() {
		// define temp variables to hold values for row parsing
		std::string camerasString;
		double x, y, z;
		double q0, q1, q2, q3;
		double ax, ay, az;
		double wx, wy, wz;
		double vx, vy, vz;

		// read the line
		mCurrentMeasurement.valid = mCsvReader.read_row(
			mCurrentMeasurement.frame,
			mCurrentMeasurement.time,
			mCurrentMeasurement.marker,
			camerasString,
			x, y, z,
			q0, q1, q2, q3,
			ax, ay, az,
			wx, wy, wz,
			vx, vy, vz
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
			mCurrentMeasurement.cameras = cameras;

			// create all the other data types
			mCurrentMeasurement.position = Point3{ x, y, z };
			mCurrentMeasurement.rotation = Rot3{ q0, q1, q2, q3 };
			mCurrentMeasurement.accel = Eigen::Vector3d{ ax, ay, az };
			mCurrentMeasurement.angVel = Eigen::Vector3d{ wx, wy, wz };
			mCurrentMeasurement.vel = Eigen::Vector3d{ vx, vy, vz };
		}

		return mCurrentMeasurement;
	}

	// output
	void SimulationContext::writeEstimate(const std::string& marker, const Point3& estimate, const Measurement& measurement) {
		std::stringstream outputLine;
		outputLine << std::to_string(measurement.frame) << ',' << std::to_string(measurement.time) << ',' << marker << ','
			<< std::to_string(measurement.position.x()) << ',' << std::to_string(measurement.position.y()) << ',' << std::to_string(measurement.position.z()) << ','
			<< std::to_string(estimate.x()) << ',' << std::to_string(estimate.y()) << ',' << std::to_string(estimate.z()) << std::endl;
		mOutputStream << outputLine.str();
	}
}