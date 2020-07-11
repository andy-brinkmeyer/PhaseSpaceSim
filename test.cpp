#include <iostream>
#include <Eigen/Eigen>

int main(int argc, char* argv[]) {
	Eigen::Matrix<double, Eigen::Dynamic, 4> mat;

	std::cout << mat.rows() << std::endl;
	mat.conservativeResize(mat.rows() + 2, Eigen::NoChange);
	Eigen::Matrix<double, 2, 4> mat2;
	mat2 << 1, 2, 3, 4, 5, 6, 7, 8;
	mat.row(0) = mat2.row(0);
	mat.row(1) = mat2.row(1);
	std::cout << mat.rows() << std::endl;
	std::cout << mat << std::endl;
}