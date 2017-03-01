/*
 * Logger.cpp
 *
 *  Created on: 10/apr/2015
 *      Author: andrea
 */

#include <Logger.h>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace utilities {

Logger::Logger() {
  lastDelta_ = 0.0;
  enable_ = true;
  useFile_ = false;
  writeOnStdOut_ = true;
}

Logger::Logger(bool writeOnStdOut) {
  lastDelta_ = 0.0;
  enable_ = true;
  useFile_ = false;
  writeOnStdOut_ = writeOnStdOut;
}

Logger::Logger(std::string fileName) {
  lastDelta_ = 0.0;
  enable_ = true;
  useFile_ = true;
  file_.open(fileName.c_str());
  writeOnStdOut_ = false;
}

Logger::Logger(bool writeOnStdOut, std::string fileName) {
  lastDelta_ = 0.0;
  enable_ = true;
  useFile_ = true;
  file_.open(fileName.c_str());
  writeOnStdOut_ = writeOnStdOut;
}

Logger::~Logger() {
}

void Logger::startEvent() {
  struct timeval tmp;
  gettimeofday(&tmp, NULL);
  eventBeginnings.push_back(tmp);
}

void Logger::startEvent(std::string message, bool newParagraph) {
  if (enable_) {
    printOn(message + " Started");
    if (newParagraph)
      newLine();
  }
  startEvent();
}

void Logger::endEvent() {
  struct timeval tmp, tmpEnd;
  gettimeofday(&tmpEnd, NULL);
  eventEnds.push_back(tmpEnd);
}

void Logger::endEvent(std::string message, bool newParagraph) {
  if (enable_) {
    printOn(message + " ended ");
    if (newParagraph)
      newLine();
  }
  endEvent();
}

void Logger::printLastTime(bool newParagraph) {
  std::stringstream s;
  computeTime();
  if (enable_) {
    s<<"Done in " << lastDelta_ << "s ";
    printOn(s.str());
    if (newParagraph)
      newLine();
  }
}

void Logger::printLastTime(std::string message, bool newParagraph) {
  std::stringstream s;
  computeTime();
  if (enable_) {
    s<<message<<"Done in " << lastDelta_ << "s ";
    printOn(s.str());
    if (newParagraph)
      newLine();
  }
}
void Logger::endEventAndPrint(bool newParagraph) {
  endEvent();
  printLastTime(newParagraph);
}

void Logger::endEventAndPrint(std::string message, bool newParagraph) {
  endEvent();
  printLastTime(message, newParagraph);
}

void Logger::enable() {
  enable_ = true;
}

void Logger::disable() {
  enable_ = false;
}

void Logger::computeTime() {
  struct timeval end, start;
  start = eventBeginnings.back();
  eventBeginnings.pop_back();
  end = eventEnds.back();
  eventEnds.pop_back();
  lastDelta_ = ((end.tv_sec - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
}

void Logger::printOn(std::string message) {

  if (writeOnStdOut_) {
    std::cout << message;
  }

  if (useFile_) {
    file_ << message;
  }
}

void Logger::newLine() {

  if (writeOnStdOut_) {
    std::cout << std::endl;
  }

  if (useFile_) {
    file_ << std::endl;
  }
}

void Logger::resetOutputPercision() {

  if (writeOnStdOut_) {
    std::cout << std::setprecision(5);
  }
  if (useFile_) {
    file_ << std::setprecision(5);
  }
}
} /* namespace utilities */

