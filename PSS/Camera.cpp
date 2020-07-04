#include <opencv2/core.hpp>
#include <Eigen/Eigen>
#include <opencv2/core/eigen.hpp>

#include "Camera.h"


namespace PSS {
	Camera::Camera(double focalLength, cv::Point2d& principalPoint, int hRes, int vRes, cv::Point3d& position, gtsam::Quaternion& rotation) {
		setFocalLength(focalLength);
		setPrincipalPoint(principalPoint);
		setHRes(hRes);
		setVRes(vRes);
		setPosition(position);
		setRotation(rotation);
		updateProjectionMatrix();
	}

	Camera::ProjectionMatrix Camera::createProjectionMatrix() {
		// create rotation part
		gtsam::Matrix3 rotMatrix{ m_rotation.matrix() };
		cv::Matx33d cvRotMatrix;
		cv::eigen2cv(rotMatrix, cvRotMatrix);

		// concatenate rotation matrix and translation vector
		cv::Matx<double, 3, 1> tVector{ -m_position };
		cv::Matx<double, 3, 4> transformMatrix;
		cv::hconcat(cvRotMatrix, tVector, transformMatrix);

		// create the intrinsic camera matrix
		cv::Matx33d intrinsicMatrix{ m_focalLength, 0, m_principalPoint.x,
												0, m_focalLength, m_principalPoint.y,
												0, 0, 1 };

		// create the projection matrix
		return (intrinsicMatrix * transformMatrix);
	}

	cv::Point2i Camera::projectPoint(cv::Point3d& point) {
		cv::Vec4d homogPoint{ point.x, point.y, point.z, 1 };
		cv::Vec3d projectedHomogPoint{ m_projectionMatrix * homogPoint };
		if (projectedHomogPoint[2] == 0.0) {
			return cv::Point2i(0, 0);
		}
		else {
			cv::Mat projectedPoint{ projectedHomogPoint[0] / projectedHomogPoint[2],
				projectedHomogPoint[1] / projectedHomogPoint[2] };

			// add the pixel mapping here
		}
	}
}
