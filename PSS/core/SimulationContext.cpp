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
		nlohmann::json json{ };
		metaStream >> json;

		// map json to structs
		std::vector<CameraConfig> cameras;
		for (nlohmann::json cameraObject : json["cameras"]) {
			Point3 cameraPosition{ 
				cameraObject["position"]["x"].get<double>(),
				cameraObject["position"]["y"].get<double>(),
				cameraObject["position"]["z"].get<double>()
			};
			Rot3 cameraRotation{
				cameraObject["rotation"]["q0"].get<double>(),
				cameraObject["rotation"]["q1"].get<double>(),
				cameraObject["rotation"]["q2"].get<double>(),
				cameraObject["rotation"]["q3"].get<double>()
			};
			Pose3 cameraPose{ cameraRotation, cameraPosition };

			// check if calibrated position and rotation are provided
			Point3 calibratedCameraPosition;
			if (cameraObject["calibratedPosition"] == nullptr) {
				calibratedCameraPosition = Point3{ cameraPosition };
			}
			else {
				calibratedCameraPosition = Point3{
				cameraObject["calibratedPosition"]["x"].get<double>(),
				cameraObject["calibratedPosition"]["y"].get<double>(),
				cameraObject["calibratedPosition"]["z"].get<double>()
				};
			}
			Rot3 calibratedCameraRotation;
			if (cameraObject["calibratedRotation"] == nullptr) {
				calibratedCameraRotation = Rot3{ cameraRotation };
			}
			else {
				calibratedCameraRotation = Rot3{
				cameraObject["calibratedRotation"]["q0"].get<double>(),
				cameraObject["calibratedRotation"]["q1"].get<double>(),
				cameraObject["calibratedRotation"]["q2"].get<double>(),
				cameraObject["calibratedRotation"]["q3"].get<double>()
				};
			}
			Pose3 calibratedCameraPose{ calibratedCameraRotation, calibratedCameraPosition };

			CameraConfig cameraConfig{
				cameraObject["id"].get<std::string>(),
				cameraPose,
				calibratedCameraPose,
				cameraObject["fieldOfView"].get<double>(),
				cameraObject["sensorWidth"].get<double>(),
				cameraObject["resolution"].get<int>(),
				cameraObject["sensorVariance"].get<double>()
			};
			cameras.push_back(cameraConfig);
		}
		mMetaData.cameras = cameras;

		std::vector<std::string> markers;
		for (int i{ 0 }; i < json["markers"].size(); i++) {
			markers.push_back(json["markers"][i].get<std::string>());
		}
		mMetaData.markers = markers;

		mMetaData.samplingRate = json["samplingRate"].get<double>();

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
	const Measurement& SimulationContext::nextMeasurement() {
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
			std::vector<std::string> cameras{ };
			char delimiter{ ';' };
			size_t pos{ 0 };
			std::string token{ };
			while ((pos = camerasString.find(delimiter)) != std::string::npos) {
				token = camerasString.substr(0, pos);
				cameras.push_back(token);
				camerasString.erase(0, pos + 1);
			}
			if (camerasString.size() > 0) {
				cameras.push_back(camerasString);
			}
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