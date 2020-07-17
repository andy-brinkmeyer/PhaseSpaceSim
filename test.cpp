#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <json/json.hpp>

int main(int argc, char* argv[]) {
	nlohmann::json j = "{ \"happy\": true, \"pi\": -1E-6 }"_json;
	std::cout << std::fixed << (double)(j["pi"]) << std::endl;
}