#include "Pose3.h"
#include "Rot3.h"

#include <Eigen/Core>


namespace PSS {
	// constructors
	Pose3::Pose3()
		: mPos{ Point3::Zero() }
	{ }

	Pose3::Pose3(const Rot3& rot3, const Point3& point3)
		: mRot{ rot3 }
		, mPos{ point3 }
	{ }

	// getters
	const Rot3& Pose3::rotation() const { return mRot; }
	const Point3& Pose3::translation() const { return mPos; }
	double Pose3::x() const { return mPos.x(); }
	double Pose3::y() const { return mPos.y(); }
	double Pose3::z() const { return mPos.z(); }

	// lin. transformation
	Point3 Pose3::transformTo(const Point3& point) const {
		RotationMatrix3 Rt{ mRot.matrix().transpose() };
		return Rt * (point - mPos);
	}
}
