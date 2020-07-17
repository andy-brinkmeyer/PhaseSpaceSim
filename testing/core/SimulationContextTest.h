#pragma once

#include <core/SimulationContext.h>

#include <gtest/gtest.h>

#include <string>
#include <fstream>
#include <cstdio>


class SimulationContextFixture : public ::testing::Test {
protected:
	// paths and file contents
	std::string metaDataPath;
	std::string measurementsPath;
	std::string outputPath;
	std::string metadataJson;
	std::string measurementsString;

	// expected values
	// metadata
	int numCameras;
	int camera1Idx;
	std::string camera1ID;
	gtsam::Point3 camera1Pos;
	gtsam::Rot3 camera1Rot;
	int camera2Idx;
	std::string camera2ID;
	gtsam::Point3 camera2Pos;
	gtsam::Rot3 camera2Rot;
	std::vector<std::string> markers;
	double samplingRate;
	double fieldOfView;
	double sensorWidth;
	double sensorVariance;

	// measurements
	int frame1;
	int frame2;
	double t1;
	double t2;
	std::string marker1;
	std::string marker2;
	std::vector<std::string> cameras1;
	std::vector<std::string> cameras2;
	gtsam::Point3 pos1;
	gtsam::Point3 pos2;
	gtsam::Rot3 rot1;
	gtsam::Rot3 rot2;
	Eigen::Vector3d accel1;
	Eigen::Vector3d accel2;
	Eigen::Vector3d angVel1;
	Eigen::Vector3d angVel2;

public:
	inline SimulationContextFixture()
		: metaDataPath{ "SimContTestMetaData.json" }
		, measurementsPath{ "SimContMeasurementsTest.csv" }
		, outputPath{ "SimContOutput.csv" }
		, metadataJson{ "{\"cameras\":"
			"[{\"id\":\"Camera_1\",\"fieldOfView\":120,\"sensorWidth\":0.1,\"sensorVariance\":1E-6,\"position\":{\"x\":0.0,\"y\":1.0,\"z\":2.0},\"rotation\":{\"q0\":1.0,\"q1\":0.0,\"q2\":0.0,\"q3\":0.0}},"
			"{\"id\":\"Camera_2\",\"fieldOfView\":120,\"sensorWidth\":0.1,\"sensorVariance\":1E-6,\"position\":{\"x\":1.0,\"y\":0.0,\"z\":0.0},\"rotation\":{\"q0\":0.0,\"q1\":0.0,\"q2\":-1.0,\"q3\":0.0}}],"
			"\"markers\":[\"Marker_1\",\"Marker_2\"],\"samplingRate\":1000}" }
		, measurementsString{ "frame,t,markerID,cameras,x,y,z,q0,q1,q2,q3,ax,ay,az,wx,wy,wz\n"
			"0,0,Marker_1,Camera_1;Camera_2,1.0,2.0,3.0,1.0,0.0,0.0,0.0,10.0,11.0,12.0,13.0,14.0,15.0\n"
			"1,0.1,Marker_2,Camera_2,0.0,1.0,2.0,0.0,-1.0,0.0,0.0,9.0,10.0,11.0,12.0,13.0,14.0\n" }
		, numCameras{ 2 }
		, camera1Idx{ 0 }, camera1ID{ "Camera_1" }, camera1Pos{ 0.0, 1.0, 2.0 }, camera1Rot{ 1.0, 0.0, 0.0, 0.0 }
		, camera2Idx{ 1 }, camera2ID{ "Camera_2" }, camera2Pos{ 1.0, 0.0, 0.0 }, camera2Rot{ 0.0, 0.0, -1.0, 0.0 }
		, markers{ "Marker_1", "Marker_2" }
		, fieldOfView{ 120 }, sensorWidth{ 0.1 }, sensorVariance{ 0.000001 }
		, samplingRate{ 1000.0 }
		, frame1{ 0 }, frame2{ 1 }
		, t1{ 0.0 }, t2{ 0.1 }
		, marker1{ "Marker_1" }, marker2{ "Marker_2" }
		, cameras1{ "Camera_1", "Camera_2" }, cameras2{ "Camera_2" }
		, pos1{ 1.0, 2.0, 3.0 }, pos2{ 0.0, 1.0, 2.0 }
		, rot1{ 1.0, 0.0, 0.0, 0.0 }, rot2{ 0.0, -1.0, 0.0, 0.0 }
		, accel1{ 10.0, 11.0, 12.0 }, accel2{ 9.0, 10.0, 11.0 }
		, angVel1{ 13.0, 14.0, 15.0 }, angVel2{ 12.0, 13.0, 14.0 }
	{
		// create dummy json
		std::remove(metaDataPath.c_str()); // make sure that the file does not already exist
		std::ofstream metaOutStream;
		metaOutStream.open(metaDataPath);
		metaOutStream << metadataJson;
		metaOutStream.close();

		// create dummy csv measurements
		remove(measurementsPath.c_str()); // make sure that the file does not already exist
		std::ofstream csvOutStream;
		csvOutStream.open(measurementsPath);
		csvOutStream << measurementsString;
		csvOutStream.close();
	}

	inline ~SimulationContextFixture() {
		std::remove(metaDataPath.c_str());
		std::remove(measurementsPath.c_str());
		std::remove(outputPath.c_str());
	}
};