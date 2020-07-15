#include <fstream>
#include <json/json.hpp>
#include <iostream>
#include <map>
#include <vector>

int main(int argc, char* argv[]) {
	nlohmann::json j;

	std::ifstream f{ "C:\\Users\\andyb\\Projekte\\UCL\\msc_project\\code\\matlab\\input\\meta.json" };

	f >> j;

	std::cout << j["samplingRate"] + 100;
}