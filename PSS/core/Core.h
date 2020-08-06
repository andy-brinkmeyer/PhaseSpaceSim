#pragma once

#include "../camera/Camera.h"
#include "SimulationContext.h"
#include "../geometry/Pose3.h"

#include <unordered_map>
#include <vector>
#include <string>
#include <exception>


namespace PSS {
	/**
	 * \brief Unordered map that maps the camera ID to a Camera instance.
	*/
	typedef std::unordered_map<std::string, Camera> CameraMap;

	/**
	 * \brief This class represents the motion capture core system.
	 *
	 * The Core class holds the cameras that make up the motion capture system and provides a basic estimation algorithm.
	*/
	class Core {
		CameraMap mCameras;

	public:
		// cosntructors
		/**
		 * \brief Constructor from CameraMap.
		 *
		 * \param cameras CameraMap that holds the camera instances of the motion capture setup.
		*/
		Core(const CameraMap& cameras);

		/**
		 * \brief Cosntructor from a SimulationContext.
		 *
		 * \param simContext SimulationContext that holds the information about the motion capture system setup and the simulated measurements.
		*/
		Core(SimulationContext& simContext);

		// getters
		const CameraMap& cameras() const; /**< Returns the CameraMap of the system. */

		// estimation
		/**
		 * \brief Estimate the position of a given point using the simulated motion capture system.
		 *
		 * First the point is projected onto the individual \link LinearDetector LinearDetectors \endlink. Then, if not disbled, noise is added.
		 * Then the estimation equations are created and assembled into an estimation matrix. Finally SVD is used to find the least square solution, if one exists.
		 *
		 * If the system is underdetermined an UnderdeterminedSystem exception is thrown.
		 *
		 * \param point True coordinates of the point to be estimated.
		 * \param cameras List of cameras (camera IDs) from which the marker is visible.
		 * \param addSensorNoise Add zero-mean Gaussian noise to the true sensor measurements.
		*/
		Point3 estimateFromCameras(const Point3& point, const std::vector<std::string>& cameras, bool addSensorNoise = true) const;

		/**
		 * \brief Estimate the position of a given Measurement using the simulated motion capture system.
		 *
		 * First the point in the Measurement is projected onto the individual \link LinearDetector LinearDetectors \endlink. Then, if not disbled, noise is added.
		 * Then the estimation equations are created and assembled into an estimation matrix. Finally SVD is used to find the least square solution, if one exists.
		 *
		 * If the system is underdetermined an UnderdeterminedSystem exception is thrown.
		 *
		 * \param measurement Measurement to be estimated.
		 * \param addSensorNoise Add zero-mean Gaussian noise to the true sensor measurements.
		*/
		Point3 estimateFromCameras(const Measurement& measurement, bool addSensorNoise = true) const;

		/**
		 * \brief This is a convenience function that iterates through all the \link Measurement Measurements \endlink of a SimulationContext,
		 * estimates their position and writes the result to the output specified by the SimulationContext.
		 *
		 * \param simContext SimulationContext that holds the \link Measurement Measurements \endlink to estimate.
		 * \param addSensorNoise Add zero-mean Gaussian noise to the true sensor measurements.
		*/
		void simulateCameraOnly(SimulationContext& simContext, bool addSensorNoise = true);
	};

	// custom exception
	/**
	 * \brief Custom exception that is thrown if a linear system is underdetermined.
	*/
	class UnderdeterminedSystem : public std::exception {
		std::string mMsg;

	public:
		/**
		 * \brief Constructor from message.
		 *
		 * \param msg Error message.
		*/
		UnderdeterminedSystem(const std::string& msg);

		/**
		 * \brief Returns explanatory string.
		*/
		virtual const char* what() const throw ();
	};
}