/*
 * CameraPointsCollection.h
 *
 *  Created on: 25 Mar 2016
 *      Author: enrico
 */

#ifndef INCLUDE_TEST_CAMERAPOINTSCOLLECTION_H_
#define INCLUDE_TEST_CAMERAPOINTSCOLLECTION_H_

#include <types_reconstructor.hpp>
#include <map>

class CameraPointsCollection {
public:
	CameraPointsCollection();
	virtual ~CameraPointsCollection();

	int hasCamera(long unsigned int);
	void addCamera(CameraType*);
	CameraType* getCamera(long unsigned int);
	int numCameras();

	int hasPoint(long unsigned int);
	void addPoint(PointType*);
	PointType* getPoint(long unsigned int);
	int numPoints();

	void addVisibility(long unsigned int, long unsigned int);
	void addVisibility(CameraType* camera, PointType* point);

	void clear();

	const std::map<unsigned long int, CameraType*>& getCameras() const {
		return cameras_;
	}

	const std::map<unsigned long int, PointType*>& getPoints() const {
		return points_;
	}

private:
	std::map<long unsigned int, PointType*> points_;
	std::map<long unsigned int, CameraType*> cameras_;

	// std::vector<std::vector<int> > camViewingPointN_;
	// std::vector<std::vector<int> > pointsVisibleFromCamN_;
	// std::vector<std::vector<glm::vec2> > point2DoncamViewingPoint_;
};

#endif /* INCLUDE_TEST_CAMERAPOINTSCOLLECTION_H_ */
