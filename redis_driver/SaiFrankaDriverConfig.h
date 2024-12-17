#pragma once

#include <string>
#include <stdexcept>
#include "tinyxml2.h"

namespace Sai {
namespace Franka {

enum class RobotType {
	FRANKA_PANDA,
	FRANKA_RESEARCH_3
};

struct DriverConfig {
	std::string robot_name;
	std::string robot_ip;
	std::string redis_prefix = "sai";
	RobotType robot_type = RobotType::FRANKA_RESEARCH_3;
	bool use_conservative_bounds = false;
	bool verbose = true;
};

DriverConfig loadConfig(const std::string& config_file) {
	DriverConfig config;
	
	// load config file
	tinyxml2::XMLDocument doc;
	if (doc.LoadFile(config_file.c_str()) != tinyxml2::XML_SUCCESS) {
		throw runtime_error("Could not load driver config file: " +
							config_file);
	}

	// parse driver config
	tinyxml2::XMLElement* driver_xml = doc.FirstChildElement("saiFrankaDriverConfig");
	if (driver_xml == nullptr) {
		throw runtime_error(
			"No 'saiFrankaDriverConfig' element found in driver config file: " +
			config_file);
	}

	// robot name
	if(!driver_xml->Attribute("robotName")) {
		throw runtime_error("No 'robotName' attribute found in driver config file: " +
							config_file);
	}
	config.robot_name = driver_xml->Attribute("robotName");

	// robot ip
	if(!driver_xml->Attribute("robotIP")) {
		throw runtime_error("No 'robotIP' attribute found in driver config file: " +
							config_file);
	}
	config.robot_ip = driver_xml->Attribute("robotIP");

    // redis prefix
    if(driver_xml->Attribute("redisPrefix")) {
        config.redis_prefix = driver_xml->Attribute("redisPrefix");
    }

	// robot type
	if(driver_xml->Attribute("robotType")) {
		std::string robot_type_str = driver_xml->Attribute("robotType");
		if(robot_type_str == "panda") {
			config.robot_type = RobotType::FRANKA_PANDA;
		}
		else if(robot_type_str == "fr3") {
			config.robot_type = RobotType::FRANKA_RESEARCH_3;
		}
		else {
			throw runtime_error("Unknown robot type: " + robot_type_str + "\nsupported types are: Panda, fr3");
		}
	}

	// use conservative bounds
	if(driver_xml->Attribute("useConservativeBounds")) {
		config.use_conservative_bounds = driver_xml->BoolAttribute("useConservativeBounds");
	}

	// verbose
	if(driver_xml->Attribute("verbose")) {
		config.verbose = driver_xml->BoolAttribute("verbose");
	}

    return config;
}

} // namespace Franka
} // namespace Sai