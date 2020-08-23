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
	/**
	 * \brief Configuration of a single Camera.
	*/
	struct CameraConfig {
		std::string id; /**< \brief ID of the camera. */
		Pose3 pose; /**< \brief True pose of the camera. */
		Pose3 calibratedPose; /**< \brief Calibrated pose of the camera. */
		double fieldOfView; /**< \brief Field of view of the camera. */
		double sensorWidth; /**< \brief Width of the cameras sensors. */
		int resolution; /**< \brief Pixel resolution cameras sensors. */
		double sensorVariance; /**< \brief Variance of the zero-mean Gaussian sensor noise. */
	};

	/**
	 * \brief Struct holding the meta data of a motion capture simulation setup.
	*/
	struct MetaData {
		std::vector<CameraConfig> cameras; /**< \brief Vector of cameras and their configuration in the simulation setup.. */
		std::vector<std::string> markers; /**< \brief Vecotor if IDs of the markers in the simulation. */
		double samplingRate; /**< \brief Sampling rate used for simulation. */
	};

	/**
	 * \brief A single simulated inertial measurement.
	 *
	 * The measurement also contains the true pose of the marker for reference and for motion capture simulation.
	*/
	struct Measurement {
		bool valid{ false }; /**< \brief Is true if the measurement is valid, is false if not (e.g. when an input line could not be read). */
		int frame; /**< \brief Current camera frame. */
		double time; /**< \brief Time at current camera frame in seconds. */
		std::string marker; /**< \brief  ID of the marker which corresponds to the measurement. */
		std::vector<std::string> cameras; /**< \brief Vector of camera IDs from which the marker is visible. */
		Point3 position; /**< \brief True position of the marker. */
		Rot3 rotation; /**< \brief True rotation of the marker. */
		Eigen::Vector3d accel; /**< \brief Simulated acceleration of the marker. It may contain noise depending on how it was generated. */
		Eigen::Vector3d vel; /**< \brief True velocity of the marker. */
		Eigen::Vector3d angVel; /**< \brief Simulated angular velocity of the marker. It may contain noise depending on how it was generated. */
	};

	/**
	 * \brief Class representing the context in which a simulation is run.
	 *
	 * This is basically a convenience class that reads the input data and can be passed to a Core object to set up the virtual 
	 * motion capture system. It also contains some other utility functions for reading measurements and writing results.
	*/
	class SimulationContext {
		// file paths
		std::string mMetaPath;
		std::string mMeasuremtsPath;
		std::string mOutputPath;

		// extracted metadata
		MetaData mMetaData;

		// measurements
		io::CSVReader<20> mCsvReader;
		Measurement mCurrentMeasurement;

		// output
		std::ofstream mOutputStream;

	public:
		// constructors
		/**
		 * \brief Constructor from file paths.
		 *
		 * The metadata and measurements file must be in a specific format as generated from the PSS pipeline.
		 *
		 * \param metaPath Path to the .json file that holds the metadata.
		 * \param measurementsPath Path to the .csv file that contains the simulated marker kinematics.
		 * \param outputPath Path to the file where the simulation output should be written.
		*/
		SimulationContext(const std::string& metaPath, const std::string& measurementsPath, const std::string& outputPath);

		// destructor
		/**
		 * \brief Default destructor.
		*/
		~SimulationContext();

		// getters
		const MetaData& metaData(); /**< \brief returns the metadata. */
		const Measurement& currentMeasurement(); /**< \brief Returns the current measurement. */

		// read measurements
		/**
		 * \brief Read the next measurement from the input file.
		 *
		 * The parsed measurement is also stored in the object and can be obtained from the reference returned by currentMeasurement(). Hence, it suffices to 
		 * get the reference to the current measurement once and when a new measurement is read the current measurement is updated.
		 *
		 * \return The parsed measurement.
		*/
		const Measurement& nextMeasurement();

		// output
		/**
		 * \brief This is a convenience function that can be used to write a position estimate to the output file.
		 *
		 * \param marker ID of the marker whichs position was estimated.
		 * \param estimate Estimated marker position.
		 * \param measurement %Measurement from which the position was estimated.
		*/
		void writeEstimate(const std::string& marker, const Point3& estimate, const Measurement& measurement);
	};
}