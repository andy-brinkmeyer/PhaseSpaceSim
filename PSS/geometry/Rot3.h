#pragma once

#include <Eigen/Core>


namespace PSS {
	typedef Eigen::Matrix3d RotationMatrix3;

	class Rot3 {
		Eigen::Matrix3d mRot;

	public:
		// constructors
		Rot3(); // return identity rotation
		Rot3(RotationMatrix3& rotMatrix3);
		Rot3(double w, double x, double y, double z);

		// getters
		RotationMatrix3 matrix() const;
	};
}