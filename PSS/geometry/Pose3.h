#pragma once

#include "Rot3.h"

#include <Eigen/Core>


namespace PSS {
	typedef Eigen::Vector3d Point3;

	class Pose3 {
		Rot3 mRot;
		Point3 mPos;

	public:
		// constructors
		Pose3(); // standard constructor, initialized with identities
		Pose3(const Rot3& rot3, const Point3& point3);

		// getters
		const Rot3& rotation() const;
		const Point3& translation() const;
		double x() const;
		double y() const;
		double z() const;

		// lin. transformation
		Point3 transformTo(const Point3& point) const;
	};
}