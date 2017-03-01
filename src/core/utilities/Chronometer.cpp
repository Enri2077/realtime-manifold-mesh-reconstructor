/*
 * Chronometer.cpp
 *
 *  Created on: 19 Aug 2016
 *      Author: enrico
 */

#include <Chronometer.h>

Chronometer::Chronometer() {
}
Chronometer::~Chronometer() {
}

void Chronometer::start(){
	if(ticking_) return;
	ticking_ = true;

	startTime_ = std::chrono::high_resolution_clock::now();
}

void Chronometer::stop(){
	if(!ticking_) return;
	ticking_ = false;

	auto lastDelta = std::chrono::high_resolution_clock::now() - startTime_;
	elapsed_ += std::chrono::duration_cast<std::chrono::nanoseconds>(lastDelta).count();
}

void Chronometer::reset(){
	ticking_ = false;
//	startTime_ = 0;
	elapsed_ = 0;
}

long long Chronometer::getNanoseconds(){
	if(ticking_){
		stop();
		start();
	}
	return elapsed_;
}

long long Chronometer::getMicroseconds(){
	return getNanoseconds() / 1000000;
}

float Chronometer::getSeconds(){
	return getNanoseconds() / 1000000000.0;
}

