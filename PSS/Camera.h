#pragma once

#include <opencv2/core.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Rot3.h>


namespace PSS {
	class Camera {
	public:
		typedef cv::Matx<double, 3, 4> ProjectionMatrix;

		// constructors
		Camera(double focalLength, cv::Point2d& principalPoint, int hRes, int vRes, cv::Point3d& position, gtsam::Quaternion& rotation);

		// getters and setters
		double getFocalLength() { return m_focalLength; }
		void setFocalLength(double focalLength) { m_focalLength = focalLength; }

		cv::Point2d getPrincipalPoint() { return m_principalPoint; }
		void setPrincipalPoint(cv::Point2d& principalPoint) { m_principalPoint = principalPoint; }

		int getHRes() { return m_hRes; }
		void setHRes(int hRes) { m_hRes = hRes; }

		int getVRes() { return m_vRes; }
		void setVRes(int vRes) { m_vRes = vRes; }

		cv::Point3d getPosition() { return m_position; }
		void setPosition(cv::Point3d& position) { m_position = position; }

		gtsam::Rot3 getRotation() { return m_rotation; }
		void setRotation(gtsam::Quaternion& rotation) { m_rotation = gtsam::Rot3(rotation); }

		ProjectionMatrix getProjectionMatrix() { return m_projectionMatrix; }

		// methods related to projection
		void updateProjectionMatrix() { m_projectionMatrix = createProjectionMatrix(); }
		cv::Point2i projectPoint(cv::Point3d& point);

	private:
		double m_focalLength;
		cv::Point2d m_principalPoint;
		int m_hRes;
		int m_vRes;
		cv::Point3d m_position;
		gtsam::Rot3 m_rotation;
		ProjectionMatrix m_projectionMatrix;

		// funcitons related to projection
		ProjectionMatrix createProjectionMatrix();
	};
}
