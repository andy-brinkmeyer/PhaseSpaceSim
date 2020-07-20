#include "SimulationContextTest.h"

#include <core/SimulationContext.h>
#include <geometry/Rot3.h>
#include <geometry/Pose3.h>

#include <gtest/gtest.h>
#include <Eigen/Core>

#include <string>
#include <fstream>
#include <cstdio>
#include <vector>


// test fixture
class SimulationContextTest : public SimulationContextFixture { };

TEST_F(SimulationContextTest, ConstructorTest) {
	// create simulation context
	PSS::SimulationContext simContext{ metaDataPath, measurementsPath, outputPath };

	// check if json was properly parsed
	const PSS::MetaData& metaData{ simContext.metaData() };

	ASSERT_EQ(metaData.cameras.size(), numCameras);
	ASSERT_EQ(metaData.cameras[camera1Idx].id, camera1ID);
	ASSERT_EQ(metaData.cameras[camera2Idx].id, camera2ID);
	double delta{ std::abs(metaData.cameras[camera1Idx].fieldOfView - fieldOfView) };
	double epsilon{ 0.000001 };
	ASSERT_TRUE(delta < epsilon);
	delta = std::abs(metaData.cameras[camera2Idx].fieldOfView - fieldOfView);
	ASSERT_TRUE(delta < epsilon);
	delta = std::abs(metaData.cameras[camera1Idx].sensorWidth - sensorWidth);
	ASSERT_TRUE(delta < epsilon);
	delta = std::abs(metaData.cameras[camera2Idx].sensorWidth - sensorWidth);
	ASSERT_TRUE(delta < epsilon);
	epsilon = 0.000000001;
	delta = std::abs(metaData.cameras[camera1Idx].sensorVariance - sensorVariance);
	ASSERT_TRUE(delta < epsilon);
	delta = std::abs(metaData.cameras[camera2Idx].sensorVariance - sensorVariance);
	ASSERT_TRUE(delta < epsilon);

	ASSERT_TRUE(metaData.cameras[camera1Idx].pose.translation().isApprox(camera1Pos));
	ASSERT_TRUE(metaData.cameras[camera2Idx].pose.translation().isApprox(camera2Pos));
	ASSERT_TRUE(metaData.cameras[camera1Idx].pose.rotation().matrix().isApprox(camera1Rot.matrix()));
	ASSERT_TRUE(metaData.cameras[camera2Idx].pose.rotation().matrix().isApprox(camera2Rot.matrix()));

	ASSERT_EQ(metaData.markers, markers);
}

TEST_F(SimulationContextTest, NextMeasurementTest) {
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