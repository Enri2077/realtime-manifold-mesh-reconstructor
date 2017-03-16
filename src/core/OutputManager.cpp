/*
 * OutputManager.cpp
 *
 *	TODO
 */

#include <OutputManager.h>
#include <Chronometer.h>
#include <fstream>
#include <iostream>

//#include <shape_msgs/Mesh.h>
//#include <gamesh_bridge/GameshMesh.h>

OutputManager::OutputManager(Delaunay3& dt, std::map<index3, std::set<Delaunay3::Cell_handle>>& boundaryCellsSpatialMap,
		ManifoldReconstructionConfig conf) :
		dt_(dt), boundaryCellsSpatialMap_(boundaryCellsSpatialMap), conf_(conf) {

//	std::cout << "OutputManager::writeMeshToOff\t conf_:" << &conf_ << std::endl;
//	std::cout << conf_.toString() << std::endl;
}

OutputManager::~OutputManager() {
}

//void OutputManager::publishROSMesh() {
////	void OutputManager::publishROSMesh(ros::Publisher& meshPublisher) {
//
//	shape_msgs::Mesh m;
//
//	std::map<Delaunay3::Vertex_handle, int> vertexHandleToIndex;
//
//	int facetToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
//	{ 0, 2, 3 },	// facetIndex : 1
//	{ 3, 1, 0 },	// facetIndex : 2
//	{ 0, 1, 2 } 	// facetIndex : 3
//	};
//
//	// Populate the list of points, the list of vertex handles, and
//	// the associative maps from vertex handle to vertex index.
//	for (auto i_lbc : boundaryCellsSpatialMap_) { // For each spatially mapped container
//
//		for (auto c : i_lbc.second) { // For each boundary cell in the container
//
//			for (int faceIndex = 0; faceIndex < 4; faceIndex++) { // For each face in the cell
//
//				// If the face is a boundary face (between the boundary cell and a non manifold cell)
//				if (!c->neighbor(faceIndex)->info().getManifoldFlag()) {
//
//					std::array<Delaunay3::Vertex_handle, 3> triangleVertices = faceIndexToVertices(c, faceIndex);
//
//					// Add the face's vertices to the vertices list (if they aren't already in it)
//					for (auto v : triangleVertices) {
//
//						if (!vertexHandleToIndex.count(v)) {
//
//							geometry_msgs::Point p;
//							p.x = v->point().x();
//							p.y = v->point().y();
//							p.z = v->point().z();
//
//							m.vertices.push_back(p);
//							vertexHandleToIndex[v] = m.vertices.size() - 1;
//						}
//					}
//
//					// Add the face's triangle to the triangles list
//					shape_msgs::MeshTriangle t;
//					t.vertex_indices[0] = vertexHandleToIndex[triangleVertices[0]];
//					t.vertex_indices[1] = vertexHandleToIndex[triangleVertices[1]];
//					t.vertex_indices[2] = vertexHandleToIndex[triangleVertices[2]];
//
//					m.triangles.push_back(t);
//
//				}
//			}
//		}
//	}
//
//	meshPublisher_.publish(m);
////	meshPublisher.publish(m);
//
//}
//
////void OutputManager::publishROSColoredMesh(ros::Publisher& meshPublisher) {
//	void OutputManager::publishROSColoredMesh() {
//
//	gamesh_bridge::GameshMesh m;
//
//	std::map<Delaunay3::Vertex_handle, int> vertexHandleToIndex;
//
//	int facetToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
//	{ 0, 2, 3 },	// facetIndex : 1
//	{ 3, 1, 0 },	// facetIndex : 2
//	{ 0, 1, 2 } 	// facetIndex : 3
//	};
//
//	// Populate the list of points, the list of vertex handles, and
//	// the associative maps from vertex handle to vertex index.
//	for (auto i_lbc : boundaryCellsSpatialMap_) { // For each spatially mapped container
//
//		for (auto c : i_lbc.second) { // For each boundary cell in the container
//
//			for (int faceIndex = 0; faceIndex < 4; faceIndex++) { // For each face in the cell
//
//				// If the face is a boundary face (between the boundary cell and a non manifold cell)
//				if (!c->neighbor(faceIndex)->info().getManifoldFlag()) {
//
//					std::array<Delaunay3::Vertex_handle, 3> triangleVertices = faceIndexToVertices(c, faceIndex);
//
//					// Add the face's vertices to the vertices list (if they aren't already in it)
//					for (auto v : triangleVertices) {
//
//						if (!vertexHandleToIndex.count(v)) {
//
//							geometry_msgs::Point p;
//							p.x = v->point().x();
//							p.y = v->point().y();
//							p.z = v->point().z();
//
//							m.vertices.push_back(p);
//							vertexHandleToIndex[v] = m.vertices.size() - 1;
//
//							std_msgs::ColorRGBA color;
//							int pointId = v->info().getPointId(); //TODO type
//
//							if (pointId >= 0) {
//								PointReconstruction& point = points_->at(pointId);
//								color.r = point.r;
//								color.g = point.g;
//								color.b = point.b;
//								color.a = point.a;
//							} else {
//								color.r = 0.0;
//								color.g = 0.0;
//								color.b = 0.0;
//								color.a = 1.0;
//							}
//
//							m.vertex_colors.push_back(color);
//
//						}
//					}
//
//					// Add the face's triangle to the triangles list
//					shape_msgs::MeshTriangle t;
//					t.vertex_indices[0] = vertexHandleToIndex[triangleVertices[0]];
//					t.vertex_indices[1] = vertexHandleToIndex[triangleVertices[1]];
//					t.vertex_indices[2] = vertexHandleToIndex[triangleVertices[2]];
//
//					m.triangles.push_back(t);
//
//				}
//			}
//		}
//	}
//
//	meshPublisher_.publish(m);
////	meshPublisher.publish(m);
//
//}

//void OutputManager::collectMesh(std::vector<shape_msgs::MeshTriangle>& triangles, std::vector<geometry_msgs::Point>& vertices, std::vector<std_msgs::ColorRGBA>& vertex_colors){
//
//}

void OutputManager::writeMeshToOff(const std::string filename) {

	std::cout << "OutputManager::writeMeshToOff\t conf_:" << &conf_ << std::endl;
	std::cout << conf_.toString() << std::endl;

	Chronometer chrono1, chrono2;
	chrono1.start();

	int avoidedSteinerVertices = 0;

	std::ofstream outfile;
	std::vector<PointD3> points;
	std::set<index3> triangles;
	std::map<Delaunay3::Vertex_handle, int> vertexHandleToIndex;
	std::vector<Delaunay3::Vertex_handle> vertexHandles;

	int facetToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
	{ 0, 2, 3 },	// facetIndex : 1
	{ 3, 1, 0 },	// facetIndex : 2
	{ 0, 1, 2 } 	// facetIndex : 3
	};

	outfile.open(filename.c_str());
	if (!outfile.is_open()) {
		std::cerr << "Unable to open file: " << filename << std::endl;
		return;
	}

	std::cout << "boundaryCellsSpatialMap size\t\t " << boundaryCellsSpatialMap_.size() << std::endl;

	std::set<Delaunay3::Cell_handle> boundaryCells;

	for (auto i_lbc : boundaryCellsSpatialMap_) { // For each spatially mapped container

		for (auto c : i_lbc.second) { // For each boundary cell in the container
			boundaryCells.insert(c);
		}
	}

	// Populate the list of points, the list of vertex handles, and
	// the associative maps from vertex handle to vertex index).
//	for (auto i_lbc : boundaryCellsSpatialMap_) { // For each spatially mapped container
//
//		for (auto c : i_lbc.second) { // For each boundary cell in the container

	for (auto c : boundaryCells) {
		if(!dt_.is_cell(c)){
			std::cerr << "OutputManager::writeMeshToOff:\t\tdead cell in boundary" << std::endl;
			continue;
		}
		
	
		for (int faceIndex = 0; faceIndex < 4; faceIndex++) { // For each face in the cell

			// If the face is a boundary face (between the boundary cell and a non manifold cell)
			if (!c->neighbor(faceIndex)->info().getManifoldFlag()) {


				std::array<Delaunay3::Vertex_handle, 3> triangleVertices = faceIndexToVertices(c, faceIndex);

				int steinerVertices = 0;
				for (auto v : triangleVertices) if(v->info().getPointId() < 0) steinerVertices++;
				
//				std::cout << "steiner vertices:\t\t" << steinerVertices << "\tconf_.maxSteinerVerticesPerTriangle: " << conf_.maxSteinerVerticesPerTriangle << std::endl;

				if(conf_.maxSteinerVerticesPerTriangle >= 0){
					if(steinerVertices >= conf_.maxSteinerVerticesPerTriangle) {
//						std::cout << "steiner vertices:\t\t" << steinerVertices << "\tconf_.maxSteinerVerticesPerTriangle: " << conf_.maxSteinerVerticesPerTriangle << std::endl;
//						std::cout << "avoided steiner vertices:\t" << triangleVertices[0]->info().getPointId() << ", " << triangleVertices[1]->info().getPointId() << ", " << triangleVertices[2]->info().getPointId() << ", " << std::endl;
						avoidedSteinerVertices++;
						continue;
					}
				}else{
					std::cerr << "s";//conf_.toString() << std::endl;	
				}

				// Add the face's vertices to the vertices list (if they aren't already in it)
				for (auto v : triangleVertices) {

					if (!vertexHandleToIndex.count(v)) {
						points.push_back(v->point());
						vertexHandles.push_back(v);
						vertexHandleToIndex[v] = points.size() - 1;
					}
				}

				// Add the face's triangle to the triangles list
				triangles.insert(
						index3(vertexHandleToIndex[triangleVertices[0]], vertexHandleToIndex[triangleVertices[1]],
								vertexHandleToIndex[triangleVertices[2]]));

			}
		}
	}

	chrono1.stop();
	chrono2.start();

	outfile << "OFF" << std::endl << points.size() << " " << triangles.size() << " 0" << std::endl;

	for (auto p : points)
		outfile << static_cast<float>(p.x()) << " " << static_cast<float>(p.y()) << " " << static_cast<float>(p.z()) << " " << std::endl;

	for (auto t : triangles)
		outfile << "3 " << t.i << " " << t.j << " " << t.k << std::endl;

	outfile.close();

	chrono2.stop();
	std::cout << "writeMeshToOff collect  :\t\t" << chrono1.getSeconds() << " s" << std::endl;
	std::cout << "writeMeshToOff write    :\t\t" << chrono2.getSeconds() << " s" << std::endl;
	std::cout << "writeMeshToOff vertices :\t" << points.size() << std::endl;
	std::cout << "writeMeshToOff triangles:\t" << triangles.size() << std::endl;
	std::cout << "writeMeshToOff avoidedSteinerVertices:\t" << avoidedSteinerVertices << std::endl;

}

std::array<Delaunay3::Vertex_handle, 3> OutputManager::faceIndexToVertices(Delaunay3::Cell_handle c, int faceIndex) {
	std::array<Delaunay3::Vertex_handle, 3> vertices;

	int faceToTriangleMatrix[4][3] = { { 3, 2, 1 },	// facetIndex : 0
	{ 0, 2, 3 },	// faceIndex : 1
	{ 3, 1, 0 },	// faceIndex : 2
	{ 0, 1, 2 } 	// faceIndex : 3
	};

	for (int i = 0; i < 3; i++)
		vertices[i] = c->vertex(faceToTriangleMatrix[faceIndex][i]);

	return vertices;
}


void OutputManager::writeAllVerticesToOFF(std::string pathPrefix, std::vector<int> ids) {

	std::ofstream outputFile;
	std::ostringstream outputFileName;
	outputFileName << pathPrefix;
	for (auto id : ids)
		outputFileName << "_" << id;
	outputFileName << ".off";
	outputFile.open(outputFileName.str().c_str());

	if (!outputFile.is_open()) {
		std::cerr << "OutputManager::writeAllVerticesToOFF: Unable to open file: " << outputFileName.str() << std::endl;
		return;
	}

	std::vector<PointD3> points;

	for (auto v = dt_.vertices_begin(); v != dt_.vertices_end(); v++) {
		points.push_back(v->point());
	}

	outputFile << "OFF" << std::endl;
	outputFile << points.size() << " 0 0" << std::endl;

	// Write out lines one by one.
	for (auto p : points)
		outputFile << static_cast<float>(p.x()) << " " << static_cast<float>(p.y()) << " " << static_cast<float>(p.z()) << std::endl;

	// Close the file and return
	outputFile.close();
}

void OutputManager::writeTetrahedraToOFF(std::string pathPrefix, std::vector<int> ids, std::vector<Delaunay3::Cell_handle> & cells) {
	// Refuse to create a file with without tetrahedra
	std::ostringstream outputFileName;
	outputFileName << pathPrefix;
	for (auto id : ids)
		outputFileName << "_" << id;
	outputFileName << ".off";

	int s = cells.size();
	if (s == 0) {
		std::cerr << "OutputManager::writeTetrahedraAndRayToOFF: no tetrahedra to write in output for: " << outputFileName.str() << std::endl;
		return;
	}

	std::ofstream outputFile;
	outputFile.open(outputFileName.str().c_str());

	if (!outputFile.is_open()) {
		std::cerr << "OutputManager::writeTetrahedraToOFF: Unable to open file: " << outputFileName.str() << std::endl;
		return;
	}

	std::vector<PointD3> vertices;
	int triangleNum = 4 * cells.size();

	for (auto cell : cells) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 3; j++) {
				vertices.push_back(dt_.triangle(cell, i).vertex(j));
			}
		}
	}

	outputFile << "OFF" << std::endl;
	outputFile << vertices.size() << " " << triangleNum << " 0" << std::endl;

	for (auto v : vertices)
		outputFile << static_cast<float>(v.x()) << " " << static_cast<float>(v.y()) << " " << static_cast<float>(v.z()) << std::endl;

	for (int t = 0; t < triangleNum; t++)
		outputFile << "3 " << 3 * t + 0 << " " << 3 * t + 1 << " " << 3 * t + 2 << std::endl;

	outputFile.close();
}

void OutputManager::writeTetrahedraToOFF(std::string pathPrefix, std::vector<int> ids, std::set<Delaunay3::Cell_handle> & cells) {
	// Refuse to create a file with without tetrahedra
	std::ostringstream outputFileName;
	outputFileName << pathPrefix;
	for (auto id : ids)
		outputFileName << "_" << id;
	outputFileName << ".off";

	int s = cells.size();
	if (s == 0) {
		std::cerr << "OutputManager::writeTetrahedraAndRayToOFF: no tetrahedra to write in output for: " << outputFileName.str() << std::endl;
		return;
	}

	std::ofstream outputFile;
	outputFile.open(outputFileName.str().c_str());

	if (!outputFile.is_open()) {
		std::cerr << "OutputManager::writeTetrahedraToOFF: Unable to open file: " << outputFileName.str() << std::endl;
		return;
	}

	std::vector<PointD3> vertices;
	int triangleNum = 4 * cells.size();

	for (auto cell : cells) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 3; j++) {
				vertices.push_back(dt_.triangle(cell, i).vertex(j));
			}
		}
	}

	outputFile << "OFF" << std::endl;
	outputFile << vertices.size() << " " << triangleNum << " 0" << std::endl;

	for (auto v : vertices)
		outputFile << static_cast<float>(v.x()) << " " << static_cast<float>(v.y()) << " " << static_cast<float>(v.z()) << std::endl;

	for (int t = 0; t < triangleNum; t++)
		outputFile << "3 " << 3 * t + 0 << " " << 3 * t + 1 << " " << 3 * t + 2 << std::endl;

	outputFile.close();
}

void OutputManager::writeTetrahedraAndRayToOFF(std::string pathPrefix, int cameraIndex, int pointIndex, std::vector<Delaunay3::Cell_handle> & cells, Segment constraint) {
// Refuse to create a file with without tetrahedra
	int s = cells.size();
	if (s == 0) {
		std::cerr << "OutputManager::writeTetrahedraAndRayToOFF: no tetrahedra to write in output; ray: " << cameraIndex << ", " << pointIndex << std::endl;
		return;
	}

	std::ofstream outputFile;
	std::ostringstream outputFileName;
	outputFileName << pathPrefix << "_" << cameraIndex << "_" << pointIndex << ".off";
	outputFile.open(outputFileName.str().c_str());

	if (!outputFile.is_open()) {
		std::cerr << "OutputManager::writeTetrahedraAndRayToOFF: Unable to open file: " << outputFileName.str() << std::endl;
		return;
	}

	std::vector<PointD3> vertices;
	int triangleNum = 4 * cells.size() + 1;

	for (auto cell : cells) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 3; j++) {
				vertices.push_back(dt_.triangle(cell, i).vertex(j));
			}
		}
	}

// insert the ray as a triangle degenerated to a segment
	vertices.push_back(constraint.source());
	vertices.push_back(constraint.source());
	vertices.push_back(constraint.target());

	outputFile << "OFF" << std::endl;
	outputFile << vertices.size() << " " << triangleNum << " 0" << std::endl;

	for (auto v : vertices)
		outputFile << static_cast<float>(v.x()) << " " << static_cast<float>(v.y()) << " " << static_cast<float>(v.z()) << std::endl;

	for (int t = 0; t < triangleNum; t++)
		outputFile << "3 " << 3 * t + 0 << " " << 3 * t + 1 << " " << 3 * t + 2 << std::endl;

	outputFile.close();
}

void OutputManager::writeTrianglesToOFF(std::string pathPrefix, std::vector<int> ids, std::vector<Delaunay3::Triangle>& triangles) {

	// Refuse to create a file with without tetrahedra
	std::ostringstream outputFileName;
	outputFileName << pathPrefix;
	for (auto id : ids)
		outputFileName << "_" << id;
	outputFileName << ".off";

	int s = triangles.size();
	if (s == 0) {
		std::cerr << "OutputManager::writeTetrahedraAndRayToOFF: no triangles to write in output for: " << outputFileName.str() << std::endl;
		return;
	}

	std::ofstream outputFile;
	outputFile.open(outputFileName.str().c_str());

	if (!outputFile.is_open()) {
		std::cerr << "OutputManager::writeTetrahedraToOFF: Unable to open file: " << outputFileName.str() << std::endl;
		return;
	}

	std::vector<PointD3> vertices;
	int triangleNum = triangles.size();

	for (auto t : triangles) {
		for (int j = 0; j < 3; j++) {
			vertices.push_back(t.vertex(j));
		}
	}

	outputFile << "OFF" << std::endl;
	outputFile << vertices.size() << " " << triangleNum << " 0" << std::endl;

	for (auto v : vertices)
		outputFile << static_cast<float>(v.x()) << " " << static_cast<float>(v.y()) << " " << static_cast<float>(v.z()) << std::endl;

	for (int t = 0; t < triangleNum; t++)
		outputFile << "3 " << 3 * t + 0 << " " << 3 * t + 1 << " " << 3 * t + 2 << std::endl;

	outputFile.close();
}

void OutputManager::writeOneTriangleAndRayToOFF(std::string pathPrefix, std::vector<int> ids, Delaunay3::Triangle & triangle, Segment constraint) {

	std::ofstream outputFile;
	std::ostringstream outputFileName;
	outputFileName << pathPrefix;
	for (auto id : ids)
		outputFileName << "_" << id;
	outputFileName << ".off";
	outputFile.open(outputFileName.str().c_str());

	if (!outputFile.is_open()) {
		std::cerr << "OutputManager::writeOneTriangleAndRayToOFF: Unable to open file: " << outputFileName.str() << std::endl;
		return;
	}

	std::vector<PointD3> vertices;
	int triangleNum = 2;

	for (int j = 0; j < 3; j++) {
		vertices.push_back(triangle.vertex(j));
	}

// insert the ray as a triangle degenerated to a segment
	vertices.push_back(constraint.source());
	vertices.push_back(constraint.source());
	vertices.push_back(constraint.target());

	outputFile << "OFF" << std::endl;
	outputFile << vertices.size() << " " << triangleNum << " 0" << std::endl;

	for (auto v : vertices)
		outputFile << static_cast<float>(v.x()) << " " << static_cast<float>(v.y()) << " " << static_cast<float>(v.z()) << std::endl;

	for (int t = 0; t < triangleNum; t++)
		outputFile << "3 " << 3 * t + 0 << " " << 3 * t + 1 << " " << 3 * t + 2 << std::endl;

	outputFile.close();
}

void OutputManager::writeRaysToOFF(std::string pathPrefix, std::vector<int> ids, std::vector<Segment>& constraints) {

	std::ofstream outputFile;
	std::ostringstream outputFileName;
	outputFileName << pathPrefix << "/all_rays/rays";
	for (auto id : ids)
		outputFileName << "_" << id;
	outputFileName << ".off";
	outputFile.open(outputFileName.str().c_str());

	if (!outputFile.is_open()) {
		std::cerr << "OutputManager::writeRaysToOFF: Unable to open file: " << outputFileName.str() << std::endl;
		return;
	}

	std::vector<PointD3> vertices;
	int triangleNum = constraints.size();

	// insert the rays as triangles degenerated to segments
	for (auto constraint : constraints) {
		vertices.push_back(constraint.source());
		vertices.push_back(constraint.source());
		vertices.push_back(constraint.target());
	}

	outputFile << "OFF" << std::endl;
	outputFile << vertices.size() << " " << triangleNum << " 0" << std::endl;

	for (auto v : vertices)
		outputFile << static_cast<float>(v.x()) << " " << static_cast<float>(v.y()) << " " << static_cast<float>(v.z()) << std::endl;

	for (int t = 0; t < triangleNum; t++)
		outputFile << "3 " << 3 * t + 0 << " " << 3 * t + 1 << " " << 3 * t + 2 << std::endl;

	outputFile.close();
}
