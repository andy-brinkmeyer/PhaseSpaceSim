#include "Camera.h"
#include <iostream>

int main(int argc, char* argv[]) {
	double focalLength = 0.05f;
	cv::Point2d principalPoint{ 1, 1 };
	int hRes = 1920;
	int vRes = 1080;
	cv::Point3d position{ 1, 2, 3 };
	gtsam::Quaternion rotation{ 1, 0, 0, 0 };
	PSS::Camera camera{ focalLength, principalPoint, hRes, vRes, position, rotation };

	std::cout << camera.getPosition();
}