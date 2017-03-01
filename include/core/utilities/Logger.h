/*
 * Logger.h
 *
 *  Created on: 10/apr/2015
 *      Author: andrea
 */

#ifndef UTILITIES_LOGGER_H_
#define UTILITIES_LOGGER_H_

#include <sys/time.h>
#include <vector>
#include <string>
#include <fstream>

namespace utilities {

class Logger {
public:
  Logger();
  Logger(bool writeOnStdOut);
  Logger(std::string fileName);
  Logger(bool writeOnStdOut, std::string fileName);

  virtual ~Logger();
  void resetOutputPercision();
  void enable();
  void disable();
  void startEvent();
  void startEvent(std::string message, bool newParagraph = false);
  void endEvent();
  void endEvent(std::string message, bool newParagraph = false);
  void printLastTime(bool newParagraph = false);
  void printLastTime(std::string message, bool newParagraph = false);
  void endEventAndPrint(bool newParagraph = false);
  void endEventAndPrint(std::string message, bool newParagraph = false);

  float getLastDelta() const {
    return lastDelta_;
  }

  void setWriteOnStdOut(bool writeOnStdOut) {
    writeOnStdOut_ = writeOnStdOut;
  }

  void setFileName(std::string fileName) {
    file_.open(fileName.c_str());
    useFile_ = true;
  }

private:
  void computeTime();
  void printOn(std::string message);
  void newLine();
  std::vector<struct timeval> eventBeginnings, eventEnds;
  float lastDelta_;
  bool enable_;

  bool writeOnStdOut_;
  std::ofstream file_;
  bool useFile_;

};

} /* namespace utilities */

#endif /* UTILITIES_LOGGER_H_ */
