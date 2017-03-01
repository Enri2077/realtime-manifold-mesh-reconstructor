/*
 * Delaunay3DVertexInfo.h
 *
 *  Created on: 24/giu/2015
 *      Author: andrea
 */

#ifndef DELAUNAY3DVERTEXINFO_H_
#define DELAUNAY3DVERTEXINFO_H_

class Delaunay3DVertexInfo {

public:
	Delaunay3DVertexInfo();

	Delaunay3DVertexInfo(long int pointId);

	virtual ~Delaunay3DVertexInfo();

	long int getPointId() const;

	void setPointId(long int pointId);

private:
	long int pointId_ = -1;
};

#endif /* DELAUNAY3DVERTEXINFO_H_ */
