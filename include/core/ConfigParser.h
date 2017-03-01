/*
 * ConfigParser.h
 *
 *  Created on: 18 May 2016
 *      Author: enrico
 */

#include <types_config.hpp>

#include <iostream>
#include <fstream>
#include <rapidjson/document.h>

#ifndef SRC_CAM_PARSERS_CONFIGPARSER_H_
#define SRC_CAM_PARSERS_CONFIGPARSER_H_

class ConfigParser {
public:
	ConfigParser();
	virtual ~ConfigParser();

	ManifoldReconstructionConfig parse(std::string path);


};

#endif /* SRC_CAM_PARSERS_CONFIGPARSER_H_ */
