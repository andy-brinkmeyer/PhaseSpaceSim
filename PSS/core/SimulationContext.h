#pragma once

#include "../geometry/Rot3.h"
#include "../geometry/Pose3.h"

#include <json/json.hpp>
#include <csv/csv.h>
#include <Eigen/Core>

#include <string>
#include <vector>
#include <fstream>


namespace PSS {
	// structs that resemble the metadata
	struct CameraConfig {
		std::string id;
		Pose3 pose;
		Pose3 calibratedPose;
		double fieldOfView;
		double sensorWidth;
		double sensorVariance;
	};

	struct MetaData {
		std::vector<CameraConfig> cameras;
		std::vector<std::string> markers;
		double samplingRate;
	};

	// struct that resembles a measurement line
	struct Measurement {
		bool valid{ false }; // is true if the measurement is valid, is false if not (e.g. when the line could not be read)
		int frame; // camera frame
		double time; // time at current frame
		std::string marker; // marker ID
		std::vector<std::string> cameras; // vector of camera IDs from which the marker is visible
		Point3 position; // position of the marker
		Rot3 rotation; // rotation of the marker
		Eigen::Vector3d accel; // acceleration of the marker
		Eigen::Vector3d vel; // velocity of the marker
		Eigen::Vector3d angVel; // angular velocity of the marker
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

		// measurements
		io::CSVReader<20> mCsvReader;
		Measurement mCurrentMeasurement;

		// output
		std::ofstream mOutputStream;

	public:
		// constructors
		SimulationContext(const std::string& metaPath, const std::string& measurementsPath, const std::string& outputPath);

		// destructor
		~SimulationContext();

		// getters
		const MetaData& metaData();
		const Measurement& currentMeasurement();

		// read measurements
		Measurement& nextMeasurement();

		// output
		void writeEstimate(const std::string& marker, const Point3& estimate, const Measurement& measurement);
	};
}