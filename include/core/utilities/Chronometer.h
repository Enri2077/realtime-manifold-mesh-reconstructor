/*
 * Chronometer.h
 *
 *  Created on: 19 Aug 2016
 *      Author: enrico
 */

#ifndef INCLUDE_MANIFOLDRECONSTRUCTOR_CHRONOMETER_H_
#define INCLUDE_MANIFOLDRECONSTRUCTOR_CHRONOMETER_H_


#include <chrono>
#include <ctime>

class Chronometer {
public:
	Chronometer();
	virtual ~Chronometer();

	void start();
	void stop();
	void reset();

	long long getNanoseconds();
	long long getMicroseconds();
	float getSeconds();

private:
	std::chrono::high_resolution_clock::time_point startTime_;
	long long elapsed_ = 0;
	bool ticking_ = false;
};

#endif /* INCLUDE_MANIFOLDRECONSTRUCTOR_CHRONOMETER_H_ */
