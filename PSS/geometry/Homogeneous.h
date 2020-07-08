#pragma once

#include <Eigen/Eigen>


namespace PSS {
	template<typename _scalar, int _length>
	class PointH {
	public:
		using HomogeneousPoint = Eigen::Matrix<_scalar, _length, 1>;
		using InhomogeneousPoint = Eigen::Matrix<_scalar, _length - 1, 1>;

		// constructors
		PointH(HomogeneousPoint point) { mPoint = point; }
		PointH(InhomogeneousPoint point) { mPoint << point, 1; }

		// getters
		HomogeneousPoint point() { return mPoint; }

	private:
		HomogeneousPoint mPoint;
	};
}