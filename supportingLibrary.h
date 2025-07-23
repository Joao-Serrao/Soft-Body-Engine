#ifndef SUPPORTINGLIBRARY_H
#define SUPPORTINGLIBRARY_H

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <set>

#include "structs.h"

// Calculates the volume of any model using node positions and face indices
float calculateVolume(const std::vector<glm::vec3>& nodesPos, const std::vector<unsigned int>& faces);

// Calculates the volume of a tetrahedron using node positions
float calculateTetraVolume(const Tetra& tetra, const std::vector<glm::vec3>& nodesPos);

// Calculates the current angle between two vectors
float calculateAngle(const glm::vec3& v1, const glm::vec3& v2);

// Returns the shared edge between two triangles (as pairs of 2D points)
glm::vec2 getEdge(const std::vector<glm::vec2>& edgesA, const std::vector<glm::vec2>& edgesB);

// Creates a bounding box for the entire model
void createBB(Model& model);

// Creates a bounding box from three 3D points
BB createBBPoints(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3);

// Creates a bounding box for a tetrahedron
void createBBTetra(Tetra& tetra, const std::vector<glm::vec3>& nodesPos);

// Checks if a point is inside a bounding box
bool insideBB(const BB& bb, glm::vec3 p);

// Checks if two bounding boxes intersect
bool intersectBB(const BB& bb1, const BB& bb2);

// Creates a bounding volume hierarchy (BVH) for tetrahedrons bounding boxes in the model
void createModelVBHTetra(Model& model);

// Updates the BVH for the model (e.g., after deformation)
void updateBVH(Model& model);

// Calculates and saves the normals of each vertex for lighting calculations
void computeVertexNormals(std::vector<glm::vec3>& normals, const std::vector<glm::vec3>& nodesPos, const std::vector<unsigned int>& faces);

#endif // SUPPORTINGLIBRARY_H
