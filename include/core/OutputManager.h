/*
 * OutputManager.h
 *
 *	TODO
 */

#ifndef OUTPUTMANAGER_H_
#define OUTPUTMANAGER_H_

#include <types_config.hpp>
#include <types_reconstructor.hpp>
#include <string>
#include <array>
#include <map>
#include <set>
#include <vector>

//#include <ros/ros.h>
//#include <ros/publisher.h>
//#include <geometry_msgs/Point.h>
//#include <shape_msgs/MeshTriangle.h>
//#include <std_msgs/ColorRGBA.h>

class OutputManager {
public:

	OutputManager(
			Delaunay3& dt, std::map<index3, std::set<Delaunay3::Cell_handle>>& boundaryCellsSpatialMap, ManifoldReconstructionConfig conf);
	virtual ~OutputManager();


//	void publishROSColoredMesh(ros::Publisher& meshPublisher);
//	void publishROSMesh(ros::Publisher& meshPublisher);
//	void publishROSColoredMesh();
//	void publishROSMesh();
	void writeMeshToOff(const std::string filename);

	void writeAllVerticesToOFF(std::string prefixPath, std::vector<int> ids);
	void writeTetrahedraToOFF(std::string pathPrefix, std::vector<int> ids, std::vector<Delaunay3::Cell_handle>& cells);
	void writeTetrahedraToOFF(std::string pathPrefix, std::vector<int> ids, std::set<Delaunay3::Cell_handle>& cells);
	void writeTetrahedraAndRayToOFF(std::string pathPrefix, int cameraIndex, int pointIndex, std::vector<Delaunay3::Cell_handle>& cells, Segment constraint);
	void writeTrianglesToOFF(std::string pathPrefix, std::vector<int> ids, std::vector<Delaunay3::Triangle>& triangles);
	void writeOneTriangleAndRayToOFF(std::string pathPrefix, std::vector<int> ids, Delaunay3::Triangle& triangle, Segment constraint);
	void writeRaysToOFF(std::string pathPrefix, std::vector<int> ids, std::vector<Segment>& constraints);

	void setPoints(std::vector<PointReconstruction>* points){
		points_ = points;
	}

//	void setMeshPublisher(ros::Publisher meshPublisher){
//		meshPublisher_ = meshPublisher;
//	}

private:

//	void collectMesh(std::vector<shape_msgs::MeshTriangle>& triangles, std::vector<geometry_msgs::Point>& vertices, std::vector<std_msgs::ColorRGBA>& vertex_colors);

	std::array<Delaunay3::Vertex_handle, 3> faceIndexToVertices(Delaunay3::Cell_handle c, int faceIndex);

//	ros::Publisher meshPublisher_;

	Delaunay3& dt_;
	std::map<index3, std::set<Delaunay3::Cell_handle>>& boundaryCellsSpatialMap_;
	std::vector<PointReconstruction>* points_ = NULL;

	ManifoldReconstructionConfig conf_;
};

#endif /* OUTPUTMANAGER_H_ */
