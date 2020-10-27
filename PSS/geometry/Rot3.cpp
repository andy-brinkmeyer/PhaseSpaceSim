#include "Rot3.h"

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace PSS {
	// constructors
	Rot3::Rot3()
		: mRot{ Eigen::Matrix3d::Identity() }
	{ }

	Rot3::Rot3(RotationMatrix3& rotMatrix3)
		: mRot{ rotMatrix3 }
	{ }

	Rot3::Rot3(double w, double x, double y, double z)
		: mRot{ Eigen::Quaternion(w, x, y, z).toRotationMatrix() }
	{ }

	// getters
	RotationMatrix3 Rot3::matrix() const { return mRot; }

	Eigen::Quaternion<double> Rot3::quaternion() const { return Eigen::Quaternion<double>(mRot); }
}