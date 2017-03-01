/*
 * Delaunay3DCellInfo.cpp
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#include <Delaunay3DCellInfo.h>
#include <iostream>

Delaunay3DCellInfo::Delaunay3DCellInfo() {
}

Delaunay3DCellInfo::Delaunay3DCellInfo(const Delaunay3DCellInfo& ref) {
	if (!ref.isNew())
		markOld();

	setFreeVote(ref.getFreeVote());
	setNonConicFreeVote(ref.getNonConicFreeVote());

	setIntersections(ref.getIntersections());
	setBoundaryFlag(ref.getBoundaryFlag());
	setManifoldFlag(ref.getManifoldFlag());

	setInEnclosingVolume(ref.isInEnclosingVolume(), ref.getEnclosingVersion());
	setRayTracingLastFacetIndex(ref.getRayTracingLastFacetIndex());
}

Delaunay3DCellInfo::~Delaunay3DCellInfo() {
}

// Operators (It must be assignable)
Delaunay3DCellInfo& Delaunay3DCellInfo::operator=(const Delaunay3DCellInfo& ref) {
	if (this != &ref) {

		if (!ref.isNew())
			markOld();

		setFreeVote(ref.getFreeVote());
		setNonConicFreeVote(ref.getNonConicFreeVote());

		setIntersections(ref.getIntersections());
		setBoundaryFlag(ref.getBoundaryFlag());
		setManifoldFlag(ref.getManifoldFlag());

		setInEnclosingVolume(ref.isInEnclosingVolume(), ref.getEnclosingVersion());
		setRayTracingLastFacetIndex(ref.getRayTracingLastFacetIndex());
	}

	return *this;
}


// Enclosing cache

long Delaunay3DCellInfo::getEnclosingVersion() const {
	return enclosingVersion_;
}

bool Delaunay3DCellInfo::isInEnclosingVolume() const {
	return inEnclosingVolume_;
}

void Delaunay3DCellInfo::setInEnclosingVolume(bool inEnclosingVolume, long enclosingVersion) {
	inEnclosingVolume_ = inEnclosingVolume;
	enclosingVersion_ = enclosingVersion;
}


// Raytracing cache

void Delaunay3DCellInfo::setRayTracingLastFacetIndex(int i) {
	rayTracingLastFacetIndex_ = i;
}

int Delaunay3DCellInfo::getRayTracingLastFacetIndex() const {
	return rayTracingLastFacetIndex_;
}


// Flags

bool Delaunay3DCellInfo::getBoundaryFlag() const {
	return boundary_;
}

void Delaunay3DCellInfo::setBoundaryFlag(bool value) {
	boundary_ = value;
}

bool Delaunay3DCellInfo::getManifoldFlag() const {
	return manifold_;
}

void Delaunay3DCellInfo::setManifoldFlag(bool value) {
	manifold_ = value;
}

bool Delaunay3DCellInfo::isNew() const {
	return new_;
}

void Delaunay3DCellInfo::markNew() {
	new_ = true;
}

void Delaunay3DCellInfo::markOld() {
	new_ = false;
}


// Intersections

const std::set<std::pair<int, int>>& Delaunay3DCellInfo::getIntersections() const {
	return m_setIntersections;
}

int Delaunay3DCellInfo::getNumIntersection() const {
	return m_setIntersections.size();
}

void Delaunay3DCellInfo::setIntersections(const std::set<std::pair<int, int>>& ref) {
	m_setIntersections = ref;
}

void Delaunay3DCellInfo::clearIntersections() {
	m_setIntersections.clear();
}

void Delaunay3DCellInfo::addIntersection(int cameraId, int pointId) {
	m_setIntersections.insert(m_setIntersections.end(), std::pair<int, int>(cameraId, pointId));
}

void Delaunay3DCellInfo::removeIntersection(int cameraId, int pointId) {
	m_setIntersections.erase(std::pair<int, int>(cameraId, pointId));
}


// NonConicFreeVote

int Delaunay3DCellInfo::getNonConicFreeVote() const {
	return nonConicFreeVote_;
}

void Delaunay3DCellInfo::setNonConicFreeVote(const int voteCount) {
	nonConicFreeVote_ = voteCount;
}

void Delaunay3DCellInfo::incrementNonConicFreeVote() {
	nonConicFreeVote_++;
}

void Delaunay3DCellInfo::incrementNonConicFreeVote(int num) {
	nonConicFreeVote_ += num;
}

void Delaunay3DCellInfo::decrementNonConicFreeVote() {
	if (nonConicFreeVote_ > 0)
		nonConicFreeVote_--;
}

void Delaunay3DCellInfo::decrementNonConicFreeVote(int num) {
	if (nonConicFreeVote_ - num > 0)
		nonConicFreeVote_ -= num;
	else
		nonConicFreeVote_ = 0;
}


// FreeVote

float Delaunay3DCellInfo::getFreeVote() const {
	return freeVote_;
}

void Delaunay3DCellInfo::setFreeVote(const float voteCountProb) {
	freeVote_ = voteCountProb;
}

void Delaunay3DCellInfo::incrementFreeVote(float incr) {
	freeVote_ += incr;
}

void Delaunay3DCellInfo::decrementFreeVote(float incr) {
	freeVote_ -= incr;
}


// Thresholds


bool Delaunay3DCellInfo::exceedsNonConicFreeVoteThreshold(const int threshold) const {
	return getNonConicFreeVote() >= threshold;
}

bool Delaunay3DCellInfo::isNotKeptByNonConicFreeVote(const int threshold = 1) const {
	return getNonConicFreeVote() < threshold;
}

bool Delaunay3DCellInfo::exceedsFreeVoteThreshold(const float threshold) const {
	return getFreeVote() >= threshold;
}

bool Delaunay3DCellInfo::isNotKeptByFreeVote(const float threshold = 1.0) const {
	return getFreeVote() < threshold;
}


void Delaunay3DCellInfo::lock(){
	if(!locked_) locked_ = true;
	else std::cerr << "Delaunay3DCellInfo::lock: \t\t trying to lock but already locked" << std::endl;
}

void Delaunay3DCellInfo::unlock(){
	if(locked_) locked_ = false;
	else std::cerr << "Delaunay3DCellInfo::lock: \t\t trying to unlock but already unlocked" << std::endl;
}

bool Delaunay3DCellInfo::isLocked() const {
	return locked_;
}

