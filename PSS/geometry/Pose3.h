#pragma once

#include "Rot3.h"

#include <Eigen/Core>


namespace PSS {
	/**
	 * \brief A point in three dimensional space. Represented as 3x1 matrix.
	*/
	typedef Eigen::Vector3d Point3;

	/**
	 * \brief Class representing a pose in three dimensional space.
	 *
	 * The pose consists of the rotation and position in the local frame.
	*/
	class __declspec(dllexport) Pose3 {
		Rot3 mRot;
		Point3 mPos;

	public:
		// constructors
		/**
		 * \brief Standard constructor initialized with the identities.
		 *
		 * The rotation is iinitialized as the 3x3 identity matrix. The position is initialized as zero vector.
		*/
		Pose3();

		/**
		 * \brief Constructor from a \link Rot3 rotation \endlink and \link Point3 position \endlink.
		 *
		 * \param rot3 \link Rot3 Rotation \endlink in the local frame.
		 * \param point3 \link Point3 Position \endlink in the local frame.
		*/
		Pose3(const Rot3& rot3, const Point3& point3);

		// getters
		const Rot3& rotation() const; /**< \brief Returns the rotation. */
		const Point3& translation() const; /**< \brief Returns the position/translation. */
		double x() const; /**< \brief Returns the x-position. */
		double y() const; /**< \brief Returns the y-position. */
		double z() const; /**< \brief Returns the z-position. */

		// lin. transformation
		/**
		 * \brief Maps a point from the local frame to the body frame defined by the pose.
		 *
		 * \param point \link Point3 Position \endlink in the local frame.
		 * \return The \link Point3 Position \endlink in the body frame defined by the pose.
		*/
		Point3 transformTo(const Point3& point) const;
	};
}