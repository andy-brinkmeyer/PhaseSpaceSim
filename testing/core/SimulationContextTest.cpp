#include <core/SimulationContext.h>

#include <gtest/gtest.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>

#include <string>
#include <fstream>
#include <cstdio>
#include <vector>


TEST(SimulationContextTest, ConstructorTest) {
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

	// create temporary json file
	std::string metadataJson{ "{\"cameras\":"
		"[{\"id\":\"Camera_1\",\"position\":{\"x\":0.0,\"y\":1.0,\"z\":2.0},\"rotation\":{\"q0\":1.0,\"q1\":0.0,\"q2\":0.0,\"q3\":0.0}},"
		"{\"id\":\"Camera_2\",\"position\":{\"x\":1.0,\"y\":0.0,\"z\":0.0},\"rotation\":{\"q0\":0.0,\"q1\":0.0,\"q2\":-1.0,\"q3\":0.0}}],"
		"\"markers\":[\"Marker_1\",\"Marker_2\"],\"samplingRate\":1000}" };

	std::string metaDataPath{ "SimContTestMetaData.json" };
	std::remove(metaDataPath.c_str()); // make sure that the file does not already exist
	std::ofstream outStream;
	outStream.open(metaDataPath);
	outStream << metadataJson;
	outStream.close();

	// create simulation context object
	std::string measurementsPath{ "measurements.csv" };
	std::string outputPath{ "out.csv" };

	PSS::SimulationContext simContext{ metaDataPath, measurementsPath, outputPath };

	// check if json was properly parsed
	PSS::MetaData& metaData{ simContext.metaData() };

	ASSERT_EQ(metaData.cameras.size(), numCameras);
	ASSERT_EQ(metaData.cameras[camera1Idx].id, camera1ID);
	ASSERT_EQ(metaData.cameras[camera2Idx].id, camera2ID);

	ASSERT_TRUE(metaData.cameras[camera1Idx].pose.translation().isApprox(camera1Pos));
	ASSERT_TRUE(metaData.cameras[camera2Idx].pose.translation().isApprox(camera2Pos));
	ASSERT_TRUE(metaData.cameras[camera1Idx].pose.rotation().matrix().isApprox(camera1Rot.matrix()));
	ASSERT_TRUE(metaData.cameras[camera2Idx].pose.rotation().matrix().isApprox(camera2Rot.matrix()));

	ASSERT_EQ(metaData.markers, markers);

	// cleanup
	std::remove(metaDataPath.c_str());
}