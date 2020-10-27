#pragma once

#include "../../PSS_EXPORT.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace PSS {
	/**
	 * \brief 3x3 rotation matrix.
	*/
	typedef Eigen::Matrix3d RotationMatrix3;

	/**
	 * \brief Class that represents a rotatin in three dimensional space.
	*/
	class PSS_EXPORT Rot3 {
		Eigen::Matrix3d mRot;

	public:
		// constructors
		/**
		 * \brief Default constructor that initializes an identity rotation, i.e. 3x3 identity matrix.
		*/
		Rot3();

		/**
		 * \brief Constructor from a rotation matrix.
		 * 
		 * \param rotMatrix3 Rotation matrix that repesents the mapping of a vector from body to local frame.
		*/
		Rot3(RotationMatrix3& rotMatrix3);

		/**
		 * \brief Constructor from a unit quaternion.
		 *
		 * The rotation can be constructed from a rotation quaternion of the form \f$ q = w + x \,\mathbf{i} + y \,\mathbf{j} + z \,\mathbf{k} \f$.
		 *
		 * \warning The quaternion must be normalized. If not undefined behaviour might occur.
		 *
		 * \param w part of the quaternion.
		 * \param x part of the quaternion.
		 * \param y part of the quaternion.
		 * \param z part of the quaternion.
		*/
		Rot3(double w, double x, double y, double z);

		// getters
		RotationMatrix3 matrix() const; /**< \brief Returns the rotation as rotation matrix. */
		Eigen::Quaternion<double> quaternion() const; /**< \brief Returns the rotation as quaternion. */
	};
}
