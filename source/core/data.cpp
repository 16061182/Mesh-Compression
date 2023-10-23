#include "data.h"

#include <fstream>
#include <nlohmann/json.hpp>

Config::Config(std::string json_file) {
	std::ifstream f(json_file);
	nlohmann::json config = nlohmann::json::parse(f);

	atoms = config["atoms"];
	N_bins = config["N_bins"];
	patch_size_limit = config["patch_size_limit"];
	patch_normal_tolerance = config["patch_normal_tolerance"];
	float_precision = config["float_precision"];
}
