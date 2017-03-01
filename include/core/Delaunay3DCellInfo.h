/*
 * Delaunay3DCellInfo.h
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#ifndef DELAUNAY3DCELLINFO_H_
#define DELAUNAY3DCELLINFO_H_

#include <set>
#include <utility>

class Delaunay3DCellInfo {
public:

	// Constructors (it must be default-constructable)
	Delaunay3DCellInfo();
	Delaunay3DCellInfo(const Delaunay3DCellInfo& ref);
	virtual ~Delaunay3DCellInfo();
	Delaunay3DCellInfo& operator=(const Delaunay3DCellInfo& rhs);

	long getEnclosingVersion() const;
	bool isInEnclosingVolume() const;
	void setInEnclosingVolume(bool inEnclosingVolume, long enclosingVersion);

	int getRayTracingLastFacetIndex() const;
	void setRayTracingLastFacetIndex(int i);

	bool getBoundaryFlag() const;
	void setBoundaryFlag(bool value);

	bool getManifoldFlag() const;
	void setManifoldFlag(bool value);

	bool isNew() const;
	void markNew();
	void markOld();

	const std::set<std::pair<int, int>>& getIntersections() const;
	int getNumIntersection() const;
	void setIntersections(const std::set<std::pair<int, int>>& ref);
	void clearIntersections();
	void addIntersection(int cameraId, int pointId);
	void removeIntersection(int cameraId, int pointId);

	int getNonConicFreeVote() const;
	void setNonConicFreeVote(const int voteCount);
	void incrementNonConicFreeVote();
	void incrementNonConicFreeVote(int num);
	void decrementNonConicFreeVote();
	void decrementNonConicFreeVote(int num);

	float getFreeVote() const;
	void setFreeVote(const float voteCountProb);
	void incrementFreeVote(float incr);
	void decrementFreeVote(float incr);

	bool exceedsNonConicFreeVoteThreshold(const int threshold) const;
	bool isNotKeptByNonConicFreeVote(const int threshold) const;
	bool exceedsFreeVoteThreshold(const float threshold) const;
	bool isNotKeptByFreeVote(const float threshold) const;

	bool isLocked() const;
	void lock();
	void unlock();

private:

	std::set<std::pair<int, int>> m_setIntersections;
	int nonConicFreeVote_ = 0;
	float freeVote_ = 0.0;

	bool new_ = true;
	bool boundary_ = false;
	bool manifold_ = false;

	long enclosingVersion_ = -1;
	bool inEnclosingVolume_ = false;

	int rayTracingLastFacetIndex_ = -1;
	
	bool locked_ = false;

};

#endif /* DELAUNAY3DCELLINFO_H_ */
