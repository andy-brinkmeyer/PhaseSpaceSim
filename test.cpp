#include <vector>
#include <string>
#include <Eigen/Core>
#include <iostream>

int main(int argc, char* argv[]) {
	std::string camerasString{ "Cam_1;Cam_2;Cam_3" };

	std::vector<std::string> cameras;
	char delimiter{ ';' };
	size_t pos{ 0 };
	std::string token;
	while ((pos = camerasString.find(delimiter)) != std::string::npos) {
		token = camerasString.substr(0, pos);
		cameras.push_back(token);
		camerasString.erase(0, pos + 1);
	}
	cameras.push_back(camerasString);
	std::cout << cameras.size() << std::endl;
}