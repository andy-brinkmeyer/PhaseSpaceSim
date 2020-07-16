#include <core/SimulationContext.h>

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

#include <string>
#include <fstream>
#include <cstdio>
#include <vector>


// test fixture
class SimulationContextTest : public ::testing::Test {
protected:
	std::string metaDataPath;
	std::string measurementsPath;
	std::string outputPath;
	std::string metadataJson;
	std::string measurementsString;

public:
	SimulationContextTest()
		: metaDataPath{ "SimContTestMetaData.json" }
		, measurementsPath{ "SimContMeasurementsTest.csv" }
		, outputPath { "SimContOutput.csv" }
		, metadataJson{ "{\"cameras\":"
			"[{\"id\":\"Camera_1\",\"position\":{\"x\":0.0,\"y\":1.0,\"z\":2.0},\"rotation\":{\"q0\":1.0,\"q1\":0.0,\"q2\":0.0,\"q3\":0.0}},"
			"{\"id\":\"Camera_2\",\"position\":{\"x\":1.0,\"y\":0.0,\"z\":0.0},\"rotation\":{\"q0\":0.0,\"q1\":0.0,\"q2\":-1.0,\"q3\":0.0}}],"
			"\"markers\":[\"Marker_1\",\"Marker_2\"],\"samplingRate\":1000}" }
		, measurementsString{ "frame,t,markerID,cameras,x,y,z,q0,q1,q2,q3,ax,ay,az,wx,wy,wz\n"
			"0,0,Marker_1,Camera_1;Camera_2,1.0,2.0,3.0,1.0,0.0,0.0,0.0,10.0,11.0,12.0,13.0,14.0,15.0\n"
			"1,0.1,Marker_2,Camera_2,0.0,1.0,2.0,0.0,-1.0,0.0,0.0,9.0,10.0,11.0,12.0,13.0,14.0\n" }
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

	~SimulationContextTest() {
		std::remove(metaDataPath.c_str());
		std::remove(measurementsPath.c_str());
		// std::remove(outputPath.c_str());
	}
};

TEST_F(SimulationContextTest, ConstructorTest) {
	// create data for comparison
	int numCameras{ 2 };
	int camera1Idx{ 0 };
	std::string camera1ID{ "Camera_1" };
	gtsam::Point3 camera1Pos{ 0.0, 1.0, 2.0 };
	gtsam::Rot3 camera1Rot{ 1.0, 0.0, 0.0, 0.0 };
	int camera2Idx{ 1 };
	std::string camera2ID{ "Camera_2" };
	gtsam::Point3 camera2Pos{ 1.0, 0.0, 0.0 };
	gtsam::Rot3 camera2Rot{ 0.0, 0.0, -1.0, 0.0 };

	std::vector<std::string> markers{
		"Marker_1",
		"Marker_2"
	};

	double samplingRate{ 1000.0 };

	// create simulation context
	PSS::SimulationContext simContext{ metaDataPath, measurementsPath, outputPath };

	// check if json was properly parsed
	const PSS::MetaData& metaData{ simContext.metaData() };

	ASSERT_EQ(metaData.cameras.size(), numCameras);
	ASSERT_EQ(metaData.cameras[camera1Idx].id, camera1ID);
	ASSERT_EQ(metaData.cameras[camera2Idx].id, camera2ID);

	ASSERT_TRUE(metaData.cameras[camera1Idx].pose.translation().isApprox(camera1Pos));
	ASSERT_TRUE(metaData.cameras[camera2Idx].pose.translation().isApprox(camera2Pos));
	ASSERT_TRUE(metaData.cameras[camera1Idx].pose.rotation().matrix().isApprox(camera1Rot.matrix()));
	ASSERT_TRUE(metaData.cameras[camera2Idx].pose.rotation().matrix().isApprox(camera2Rot.matrix()));

	ASSERT_EQ(metaData.markers, markers);
}

TEST_F(SimulationContextTest, NextMeasurementTest) {
	// create data for comparison
	int frame1{ 0 };
	int frame2{ 1 };
	double t1{ 0.0 };
	double t2{ 0.1 };
	std::string marker1{ "Marker_1" };
	std::string marker2{ "Marker_2" };
	std::vector<std::string> cameras1{ "Camera_1", "Camera_2" };
	std::vector<std::string> cameras2{ "Camera_2" };
	gtsam::Point3 pos1{ 1.0, 2.0, 3.0 };
	gtsam::Point3 pos2{ 0.0, 1.0, 2.0 };
	gtsam::Rot3 rot1{ 1.0, 0.0, 0.0, 0.0 };
	gtsam::Rot3 rot2{ 0.0, -1.0, 0.0, 0.0 };
	Eigen::Vector3d accel1{ 10.0, 11.0, 12.0 };
	Eigen::Vector3d accel2{ 9.0, 10.0, 11.0 };
	Eigen::Vector3d angVel1{ 13.0, 14.0, 15.0 };
	Eigen::Vector3d angVel2{ 12.0, 13.0, 14.0 };

	// create simulation context
	PSS::SimulationContext simContext{ metaDataPath, measurementsPath, outputPath };

	// check if measurement is correctly initialized
	const PSS::Measurement& measurement{ simContext.currentMeasurement() };
	ASSERT_FALSE(measurement.valid);

	// test the measurements struct for the first row
	simContext.nextMeasurement();
	ASSERT_TRUE(measurement.valid);
	ASSERT_EQ(measurement.frame, frame1);
	double delta{ std::abs(measurement.time - t1) };
	double epsilon{ 0.000001 };
	ASSERT_TRUE(delta < epsilon);
	ASSERT_EQ(measurement.marker, marker1);
	ASSERT_EQ(measurement.cameras, cameras1);
	ASSERT_TRUE(measurement.position.isApprox(pos1));
	ASSERT_TRUE(measurement.rotation.matrix().isApprox(rot1.matrix()));
	ASSERT_TRUE(measurement.accel.isApprox(accel1));
	ASSERT_TRUE(measurement.angVel.isApprox(angVel1));

	// second row
	simContext.nextMeasurement();
	ASSERT_TRUE(measurement.valid);
	ASSERT_EQ(measurement.frame, frame2);
	delta = std::abs(measurement.time - t2);
	ASSERT_TRUE(delta < epsilon);
	ASSERT_EQ(measurement.marker, marker2);
	ASSERT_EQ(measurement.cameras, cameras2);
	ASSERT_TRUE(measurement.position.isApprox(pos2));
	ASSERT_TRUE(measurement.rotation.matrix().isApprox(rot2.matrix()));
	ASSERT_TRUE(measurement.accel.isApprox(accel2));
	ASSERT_TRUE(measurement.angVel.isApprox(angVel2));

	// check that end of file is correctly handled
	simContext.nextMeasurement();
	ASSERT_FALSE(measurement.valid);
}