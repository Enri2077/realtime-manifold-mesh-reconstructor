/*
 * Exceptions.hpp
 *
 *  Created on: 16 mar 2016
 *      Author: andrea
 */

#ifndef EXCEPTIONS_HPP_
#define EXCEPTIONS_HPP_

#include <stdexcept>

struct JsonParseException : public std::runtime_error
{
  JsonParseException(const std::string& msg) : std::runtime_error(msg) {}
};

struct JsonAccessException : public std::runtime_error
{
  JsonAccessException(const std::string& msg) : std::runtime_error(msg) {}
};



#endif /* EXCEPTIONS_HPP_ */
