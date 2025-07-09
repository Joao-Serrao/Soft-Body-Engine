#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/norm.hpp>

#include <GL/glew.h>
#include <GL/glut.h>
#endif

#define _USE_MATH_DEFINES
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <random>
#include <unordered_set>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <algorithm>
#include <string>
#include <cstdio>
#include <memory>
#include <set>
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "tetgen.h"


using namespace std;

float alpha = 0.0f; // Angle Right/Left
float beta = 0.0f;  // Angle Up/Down
int k = 10;  // Camera Distance
int speed = 300;  // Speed of Zoom

// Camera Position
float CameraX = 0.0f;
float CameraY = 0.0f;
float CameraZ = 0.0f;

float DX = 0;
float DY = 0;
float DZ = 0;

// Point where Camera is looking at
float LookX = 0.0f;
float LookY = 0.0f;
float LookZ = 0.0f;

int timebase = 0, frame = 0;  // FPS

// Delta Time
float currentTime = 0.0f;
float previousTime = 0.0f;

bool paused = false;  // Pausing Time
float pauseB = 0.0f;
float pauseA = 0.0f;

// Constants
const float EPSILON = 1e-6f;  // Float Precision
const float GOLDEN_RATIO = 1.618f;
const glm::vec3 ORIGIN = glm::vec3 (0.0f);

// Forces
const glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);  // Gravity vector
const glm::vec3 friction = glm::vec3(0.99f, 0.99f, 0.99f);  // General vector for friction (test)

// Struct for Vertex used in VBOs
struct Vertex {
    glm::vec3 pos;    // Position
    glm::vec3 normal; // Normal (used for light later)
};

// Struct used to define Points
struct Node {
    glm::vec3 pos   {0.0f};  // Position Vector
    glm::vec3 predicted_pos { 0.0f};  // Predicted Position Vector to used in Updating
    glm::vec3 vel   {0.0f};  // Velocity Vector
    float mass = 1.0f;  // Point Mass
    bool hasCollided = false;  // Flag to check if Point collided
    glm::vec3 normal = glm::vec3(0.0f);  // Normal of the Surface that the Point collided with
    glm::vec3 lighNormal = glm::vec3(0.0f);
};

// Struct to represent the Constraint between Nodes
struct Spring {
    int A;  // Index of First Node
    int B;  // Index of Second Node
    float compliance = 0.0f;  // Inverse Stifness (smaller = more stiff)
    float restLength = 1.0f;  // Distance between Nodes to try to achieve
    float lambda = 0.0f;  // Lambda saved between SubSteps
};

// Struct to represent the Angle Constraints between Springs
struct AngledSpring {
    int A;  // Index of the Left Point
    int B;  // Index of the Rigth Point
    glm::vec2 edge = glm::vec2(0.0f);  // Edge shared by 2 Triangles
    float compliance = 0.0f;  // Inverse Stifness (smaller = more stiff)
    float restAngle = 1.0f;  // Angle between Nodes to try to achieve
    float lambda = 0.0f;  // Lambda saved between SubSteps
};

// Struct used to represent a BB
struct BB {
    float minX = std::numeric_limits<float>::infinity(), maxX = -std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity(), maxY = -std::numeric_limits<float>::infinity();
    float minZ = std::numeric_limits<float>::infinity(), maxZ = -std::numeric_limits<float>::infinity();
};

// Struct used to represent a Tetrahedrons
struct Tetra {
    int in;  // Index of the point inside the sphere
    vector<unsigned int> out;  // Indexs of the Outer Points
    float volume;  // Volume between Nodes to try to achieve
    float lambda = 0.0f;  // Lambda saved between SubSteps
    bool surface = false;
    BB bb;
};

// Struct used to represent a BVH
struct BVH {
    BB bb;
    int self;
    int left = -1;
    int right = -1;
    int tetra;
};

// Struct used for VBOs
struct GLModel {
    GLuint vao = 0;
    GLuint vboVertices = 0;
    GLuint vboIndices = 0;
    GLsizei indexCount = 0;
};

// Struct used to represent any Model
struct Model {
    vector<Node> nodes;  // List of the Nodes (innner and outer)
    vector<unsigned int> faces;  // List of the Index that make up Faces
    vector<Spring> springs;  // List of the Constraints
    vector<AngledSpring> angledSprings;  // List of the Angle Constraints
    vector<Tetra> tetras;  // List of the Tetrahedrons
    GLModel glModel;  // Information for VBOs
    int type; // 0 - Soft; 1 - Box; 2 - Sphere
    float distV, distH, distL;  // Distance between Center and every Plane for Box type; Radius for Sphere type;
    Node center;  // Center Node
    float compliance = 0.0f;  // Inverse Stifness for Volume (smaller = more stiff)
    float lambda = 0.0f;  // Lambda saved between SubSteps
    float volume = 0.0f;  // Volume between Nodes to try to achieve
    float bounciness = 0.0f;  // Bounciness of the surface (right now is also friction)
    float subStep = 1.0f;  // Number of SubSteps per Frame
    glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f);  // Color vector
    bool wireframe = false;  // Render in Wireframe Mode
    bool update = false;  // Update Positions
    bool tetra = false;  // Uses Tetrahedrons
    BB bb;  // Defines the Bounding Box
    vector<BVH> bvhs;
};

struct Vec3Hash {
    size_t operator()(const glm::vec3& v) const {
        // Quantize to 1e-4f precision (adjust as needed)
        int x = static_cast<int>(std::round(v.x * 10000.0f));
        int y = static_cast<int>(std::round(v.y * 10000.0f));
        int z = static_cast<int>(std::round(v.z * 10000.0f));
        size_t h1 = std::hash<int>()(x);
        size_t h2 = std::hash<int>()(y);
        size_t h3 = std::hash<int>()(z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct Vec3Equal {
    bool operator()(const glm::vec3& a, const glm::vec3& b) const {
        // Same quantization used in hashing
        return std::abs(a.x - b.x) < 1e-4f &&
               std::abs(a.y - b.y) < 1e-4f &&
               std::abs(a.z - b.z) < 1e-4f;
    }
};



struct Vec3Less {
    bool operator()(const glm::vec3& a, const glm::vec3& b) const {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    }
};

vector<Model> models;  // List of Global Models
GLuint shaderProgram;

//--------------------------------------------------------------
// Forward declarations
//--------------------------------------------------------------
void   uploadModelToGPU(Model &model);
void   updateGPUpositions(Model &model);
void   drawModel(const Model &model);

// Calculates the Initial Volume of any model using Positions
float calculateVolume(Model &model) {
    vector<unsigned int> indices = model.faces;

    float volume = 0.0f;

    glm::vec origin = glm::vec3(0.0f);

    for (int i = 0; i < indices.size(); i+=3) {
        Node p1 = model.nodes[model.faces[i]];
        Node p2 = model.nodes[model.faces[i+1]];
        Node p3 = model.nodes[model.faces[i+2]];

        glm::vec3 v1 = p1.pos - origin;
        glm::vec3 v2 = p2.pos - origin;
        glm::vec3 v3 = p3.pos - origin;

        volume += glm::dot(v1, glm::cross(v2,v3))/6;
    }
    return volume;
}

// Calculates the Current Volume of any model using Predicted Positions
float calculatePredictedVolume(Model &model) {
    float volume = 0.0f;

    glm::vec origin = glm::vec3(0.0f);

    for (int i = 0; i < model.faces.size(); i+=3) {
        Node p1 = model.nodes[model.faces[i]];
        Node p2 = model.nodes[model.faces[i+1]];
        Node p3 = model.nodes[model.faces[i+2]];

        glm::vec3 v1 = p1.predicted_pos - origin;
        glm::vec3 v2 = p2.predicted_pos - origin;
        glm::vec3 v3 = p3.predicted_pos - origin;

        volume += glm::dot(v1, glm::cross(v2,v3))/6;
    }
    return volume;
}

// Calculates the Initial Volume of a Tetrahedron using Positions
float calculateTetraVolume(const Tetra &tetra, const Model &model) {
    const glm::vec3& p0 = model.nodes[tetra.out[0]].pos;
    const glm::vec3& p1 = model.nodes[tetra.out[1]].pos;
    const glm::vec3& p2 = model.nodes[tetra.out[2]].pos;
    const glm::vec3& p3 = model.nodes[tetra.in].pos;

    return glm::dot(glm::cross(p1 - p0, p2 - p0), p3 - p0) / 6.0f;
}

// Calculates the Current Volume of a Tetrahedron using Predicted Positions
float calculatePredictedTetraVolume(const Tetra &tetra, const Model &model) {
    const glm::vec3& p0 = model.nodes[tetra.out[0]].predicted_pos;
    const glm::vec3& p1 = model.nodes[tetra.out[1]].predicted_pos;
    const glm::vec3& p2 = model.nodes[tetra.out[2]].predicted_pos;
    const glm::vec3& p3 = model.nodes[tetra.in].predicted_pos;

    return glm::dot(glm::cross(p1 - p0, p2 - p0), p3 - p0) / 6.0f;
}

// Calculates Current Angle between 2 vectors
float calculateAngle(glm::vec3 v1, glm::vec3 v2) {
    v1 = glm::normalize(v1);
    v2 = glm::normalize(v2);

    float d1 = glm::length(v1);
    float d2 = glm::length(v2);

    if (d1 < 1e-6f || d2 < 1e-6f) return 0.0f;

    float dot = glm::dot(v1, v2) / (d1 * d2);

    // Clamp dot product to valid range [-1, 1] to avoid NaNs in acos
    dot = glm::clamp(dot, -1.0f, 1.0f);

    return dot;
}

// Returns the Shared Edge between 2 Triangles
glm::vec2 getEdge(vector<glm::vec2> edgesA, vector<glm::vec2> edgesB) {
    for (auto& ea : edgesA) {
        for (auto& eb : edgesB) {
            if (ea == eb) {
                return ea;
            }
        }
    }
    return glm::vec2(0);
}

void createBB(Model &model) {
    BB bb;
    for (Node &node : model.nodes) {
        bb.minX = std::min(node.pos.x, bb.minX);
        bb.minY = std::min(node.pos.y, bb.minY);
        bb.minZ = std::min(node.pos.z, bb.minZ);
        bb.maxX = std::max(node.pos.x, bb.maxX);
        bb.maxY = std::max(node.pos.y, bb.maxY);
        bb.maxZ = std::max(node.pos.z, bb.maxZ);
    }

    if (bb.minX == bb.maxX) {
        bb.minX -= 0.001f;
        bb.maxX += 0.001f;
    }
    if (bb.minY == bb.maxY) {
        bb.minY -= 0.001f;
        bb.maxY += 0.001f;
    }
    if (bb.minZ == bb.maxZ) {
        bb.minZ -= 0.001f;
        bb.maxZ += 0.001f;
    }
    model.bb = bb;
}

BB createBBPoints(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3) {
    BB bb;
    bb.minX = std::min(std::min(std::min(p1.x, bb.minX), p2.x), p3.x);
    bb.minY = std::min(std::min(std::min(p1.y, bb.minY), p2.y), p3.y);
    bb.minZ = std::min(std::min(std::min(p1.z, bb.minZ), p2.z), p3.z);
    bb.maxX = std::max(std::max(std::max(p1.x, bb.maxX),p2.x),p3.x);
    bb.maxY = std::max(std::max(std::max(p1.y, bb.maxY),p2.y),p3.y);
    bb.maxZ = std::max(std::max(std::max(p1.z, bb.maxZ),p2.z),p3.z);
    if (bb.minX == bb.maxX) {
        bb.minX -= 0.001f;
        bb.maxX += 0.001f;
    }
    if (bb.minY == bb.maxY) {
        bb.minY -= 0.001f;
        bb.maxY += 0.001f;
    }
    if (bb.minZ == bb.maxZ) {
        bb.minZ -= 0.001f;
        bb.maxZ += 0.001f;
    }
    return bb;
}

void createBBTetra(Tetra &tetra, Model &model) {
    BB bb;
    bb.minX = model.nodes[tetra.in].pos.x;
    bb.maxX = model.nodes[tetra.in].pos.x;
    bb.minY = model.nodes[tetra.in].pos.y;
    bb.maxX = model.nodes[tetra.in].pos.y;
    bb.minZ = model.nodes[tetra.in].pos.z;
    bb.maxX = model.nodes[tetra.in].pos.z;

    for (int o : tetra.out) {
        bb.minX = std::min(bb.minX, model.nodes[o].pos.x);
        bb.minY = std::min(bb.minY, model.nodes[o].pos.y);
        bb.minZ = std::min(bb.minZ, model.nodes[o].pos.z);
        bb.maxX = std::max(bb.maxX, model.nodes[o].pos.x);
        bb.maxY = std::max(bb.maxY, model.nodes[o].pos.y);
        bb.maxZ = std::max(bb.maxZ, model.nodes[o].pos.z);
    }

    if (bb.minX == bb.maxX) {
        bb.minX -= 0.001f;
        bb.maxX += 0.001f;
    }
    if (bb.minY == bb.maxY) {
        bb.minY -= 0.001f;
        bb.maxY += 0.001f;
    }
    if (bb.minZ == bb.maxZ) {
        bb.minZ -= 0.001f;
        bb.maxZ += 0.001f;
    }
    tetra.bb = bb;
}

BB addBB(BB &bb1, BB &bb2) {
    BB bb;
    bb.minX = std::min(bb1.minX, bb2.minX);
    bb.minY = std::min(bb1.minY, bb2.minY);
    bb.minZ = std::min(bb1.minZ, bb2.minZ);
    bb.maxX = std::max(bb1.maxX, bb2.maxX);
    bb.maxY = std::max(bb1.maxY, bb2.maxY);
    bb.maxZ = std::max(bb1.maxZ, bb2.maxZ);
    return bb;
}

bool insideBB(BB &bb, glm::vec3 p) {
    bool x = p.x >= bb.minX && p.x <= bb.maxX;
    bool y = p.y >= bb.minY && p.y <= bb.maxY;
    bool z = p.z >= bb.minZ && p.z <= bb.maxZ;

    if (x && y && z) {
        return true;
    }
    return false;
}

bool intersectBB(const BB &bb1, const BB &bb2) {
    bool x = bb1.maxX >= bb2.minX && bb1.minX <= bb2.maxX;
    bool y = bb1.maxY >= bb2.minY && bb1.minY <= bb2.maxY;
    bool z = bb1.maxZ >= bb2.minZ && bb1.minZ <= bb2.maxZ;

    return x && y && z;
}


void createModelVBHTetra(Model &model) {
    vector<BVH> bvhsTemp;
    vector<BVH> bvhs;

    int k = 0;

    // Create leaf BVHs
    for (int i = 0; i < model.tetras.size(); i++) {
        BVH bvh;
        createBBTetra(model.tetras[i], model);
        bvh.tetra = i;
        bvh.bb = model.tetras[i].bb;
        bvh.self = k;
        bvhs.push_back(bvh);
        bvhsTemp.push_back(bvh);
        k++;
    }

    // Agglomerative clustering: merge closest pairs
    while (bvhsTemp.size() > 1) {
        float minCost = std::numeric_limits<float>::max();
        int minI = 0, minJ = 1;

        // Find pair with minimum combined volume
        for (int i = 0; i < bvhsTemp.size(); ++i) {
            for (int j = i + 1; j < bvhsTemp.size(); ++j) {
                BB combined = addBB(bvhsTemp[i].bb, bvhsTemp[j].bb);
                float cost = (combined.maxX - combined.minX) *
                             (combined.maxY - combined.minY) *
                             (combined.maxZ - combined.minZ);
                if (cost < minCost) {
                    minCost = cost;
                    minI = i;
                    minJ = j;
                }
            }
        }

        BVH first = bvhsTemp[minI];
        BVH second = bvhsTemp[minJ];

        // Remove in reverse order to avoid index shifts
        if (minI > minJ) std::swap(minI, minJ);
        bvhsTemp.erase(bvhsTemp.begin() + minJ);
        bvhsTemp.erase(bvhsTemp.begin() + minI);

        // Create parent node
        BVH parent;
        parent.bb = addBB(first.bb, second.bb);
        parent.left = first.self;
        parent.right = second.self;
        parent.self = k;

        bvhs.push_back(parent);
        bvhsTemp.push_back(parent);
        k++;
    }

    model.bvhs = bvhs;
}
/*
void createModelVBH(Model &model) {
    vector<BVH> bvhsTemp;
    vector<BVH> bvhs;

    int k = 0;

    // Create leaf BVHs
    for (int i = 0; i < model.faces.size(); i += 3) {
        BVH bvh;
        bvh.bb = createBBPoints(
            model.nodes[model.faces[i]].pos,
            model.nodes[model.faces[i + 1]].pos,
            model.nodes[model.faces[i + 2]].pos
        );
        bvh.p1 = model.faces[i];
        bvh.p2 = model.faces[i + 1];
        bvh.p3 = model.faces[i + 2];
        bvh.self = k;
        bvhs.push_back(bvh);
        bvhsTemp.push_back(bvh);
        k++;
    }

    // Agglomerative clustering: merge closest pairs
    while (bvhsTemp.size() > 1) {
        float minCost = std::numeric_limits<float>::max();
        int minI = 0, minJ = 1;

        // Find pair with minimum combined volume
        for (int i = 0; i < bvhsTemp.size(); ++i) {
            for (int j = i + 1; j < bvhsTemp.size(); ++j) {
                BB combined = addBB(bvhsTemp[i].bb, bvhsTemp[j].bb);
                float cost = (combined.maxX - combined.minX) *
                             (combined.maxY - combined.minY) *
                             (combined.maxZ - combined.minZ);
                if (cost < minCost) {
                    minCost = cost;
                    minI = i;
                    minJ = j;
                }
            }
        }

        BVH first = bvhsTemp[minI];
        BVH second = bvhsTemp[minJ];

        // Remove in reverse order to avoid index shifts
        if (minI > minJ) std::swap(minI, minJ);
        bvhsTemp.erase(bvhsTemp.begin() + minJ);
        bvhsTemp.erase(bvhsTemp.begin() + minI);

        // Create parent node
        BVH parent;
        parent.bb = addBB(first.bb, second.bb);
        parent.left = first.self;
        parent.right = second.self;
        parent.self = k;

        bvhs.push_back(parent);
        bvhsTemp.push_back(parent);
        k++;
    }

    model.bvhs = bvhs;
}*/


void updateBVHLeafs (Model &model) {
    for (int i = 0; i < model.tetras.size(); i++) {
        createBBTetra(model.tetras[model.bvhs[i].tetra], model);
        model.bvhs[i].bb = model.tetras[model.bvhs[i].tetra].bb;
    }
}

void updateBVH (Model &model) {
    updateBVHLeafs(model);
    for (int i = model.tetras.size(); i < model.bvhs.size(); i++) {
        model.bvhs[i].bb = addBB(model.bvhs[model.bvhs[i].left].bb, model.bvhs[model.bvhs[i].right].bb);
    }
}

void computeVertexNormals(Model& model) {
    // Initialize all normals to zero
    vector<glm::vec3> normals(model.nodes.size(), glm::vec3(0.0f));
    vector<int> division(model.nodes.size(), 0);

    for (size_t i = 0; i < model.faces.size(); i += 3) {
        int i0 = model.faces[i];
        int i1 = model.faces[i + 1];
        int i2 = model.faces[i + 2];

        glm::vec3 v0 = model.nodes[i0].pos;
        glm::vec3 v1 = model.nodes[i1].pos;
        glm::vec3 v2 = model.nodes[i2].pos;

        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 faceNormal = glm::normalize(glm::cross(edge1, edge2));

        normals[i0] += faceNormal;
        division[i0] += 1;
        normals[i1] += faceNormal;
        division[i1] += 1;
        normals[i2] += faceNormal;
        division[i2] += 1;
    }

    for (size_t i = 0; i < normals.size(); ++i) {
        normals[i] /= division[i];
        model.nodes[i].lighNormal = glm::normalize(normals[i]);
    }
}


// Makes a Soft Object with the shape of a IcoSphere
Model makeSoftIcoSphere(float radius, int detail, float subStep = 1.0f, float mass = 1.0f, float bounciness = 0.0f, float volumeCompliance = 0.0f,
                    float compliance = 0.0f, float centerCompliance = 0.0f, float angleStiffness = 0.0f, float cx = 0, float cy = 0, float cz = 0,
                    glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f), bool wireframe = false, bool update = false, bool tetra = false) {
    vector<Node> vertices;
    std::unordered_map<glm::vec3, int, Vec3Hash, Vec3Equal> vertexMap;
    vector<int> faces;
    vector<Spring> springs;
    vector<Tetra> tetras;

    Model model;

    // Model Center
    Node center;
    center.pos = glm::vec3(cx, cy, cz);
    center.mass = mass;

    float a = sqrt(radius / (1 + GOLDEN_RATIO * GOLDEN_RATIO));
    float c = a * GOLDEN_RATIO;

    float x = 0.0f;
    float y = -c;
    float z = a;
    Node node;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 0;

    y = -c;
    z = -a;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 1;

    x = c;
    y = -a;
    z = 0.0f;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 2;

    x = -c;
    y = -a;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 3;

    x = a;
    y = 0.0f;
    z = c;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 4;

    x = a;
    z = -c;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 5;

    x = -a;
    z = -c;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 6;

    x = -a;
    z = c;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 7;

    x = c;
    y = a;
    z = 0.0f;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 8;

    x = -c;
    y = a;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 9;

    x = 0.0f;
    y = c;
    z = a;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 10;

    y = c;
    z = -a;
    node.pos = glm::normalize(glm::vec3(x + cx, y + cy, z + cz) - center.pos)* radius + center.pos;
    node.mass = mass;
    vertices.push_back(node);
    vertexMap[node.pos] = 11;

    faces.push_back(0); faces.push_back(1); faces.push_back(2);
    faces.push_back(1); faces.push_back(0); faces.push_back(3);
    faces.push_back(0); faces.push_back(2); faces.push_back(4);
    faces.push_back(2); faces.push_back(1); faces.push_back(5);
    faces.push_back(1); faces.push_back(3); faces.push_back(6);
    faces.push_back(3); faces.push_back(0); faces.push_back(7);
    faces.push_back(0); faces.push_back(4); faces.push_back(7);
    faces.push_back(1); faces.push_back(6); faces.push_back(5);
    faces.push_back(4); faces.push_back(2); faces.push_back(8);
    faces.push_back(2); faces.push_back(5); faces.push_back(8);
    faces.push_back(6); faces.push_back(3); faces.push_back(9);
    faces.push_back(3); faces.push_back(7); faces.push_back(9);
    faces.push_back(7); faces.push_back(4); faces.push_back(10);
    faces.push_back(5); faces.push_back(6); faces.push_back(11);
    faces.push_back(4); faces.push_back(8); faces.push_back(10);
    faces.push_back(8); faces.push_back(5); faces.push_back(11);
    faces.push_back(6); faces.push_back(9); faces.push_back(11);
    faces.push_back(9); faces.push_back(7); faces.push_back(10);
    faces.push_back(10); faces.push_back(8); faces.push_back(11);
    faces.push_back(11); faces.push_back(9); faces.push_back(10);

    for (int i = 0; i < detail; i++) {
        vector<int> tempFaces;
        for (int j = 0; j < faces.size(); j+=3) {
            Node v1 = vertices[faces[j]];
            Node v2 = vertices[faces[j+1]];
            Node v3 = vertices[faces[j+2]];

            Node m1;
            glm::vec3 midpoint1 = (v2.pos + v1.pos) * 0.5f;
            m1.pos = glm::normalize(midpoint1 - center.pos) * radius + center.pos;
            m1.mass = mass;

            Node m2;
            glm::vec3 midpoint2 = (v3.pos + v2.pos) * 0.5f;
            m2.pos = glm::normalize(midpoint2 - center.pos) * radius + center.pos;
            m2.mass = mass;

            Node m3;
            glm::vec3 midpoint3 = (v1.pos + v3.pos) * 0.5f;
            m3.pos = glm::normalize(midpoint3 - center.pos) * radius + center.pos;
            m3.mass = mass;

            int i1, i2, i3;

            auto it = vertexMap.find(m1.pos);
            if (it == vertexMap.end()) {
                i1 = vertices.size();
                vertices.push_back(m1);
                vertexMap[m1.pos] = i1;
            } else {
                i1 = it->second;
            }

            it = vertexMap.find(m2.pos);
            if (it == vertexMap.end()) {
                i2 = vertices.size();
                vertices.push_back(m2);
                vertexMap[m2.pos] = i2;
            } else {
                i2 = it->second;
            }

            it = vertexMap.find(m3.pos);
            if (it == vertexMap.end()) {
                i3 = vertices.size();
                vertices.push_back(m3);
                vertexMap[m3.pos] = i3;
            } else {
                i3 = it->second;
            }


            tempFaces.push_back(faces[j]);     tempFaces.push_back(i1);     tempFaces.push_back(i3);
            tempFaces.push_back(i1);           tempFaces.push_back(faces[j + 1]); tempFaces.push_back(i2);
            tempFaces.push_back(i3);           tempFaces.push_back(i2);     tempFaces.push_back(faces[j + 2]);
            tempFaces.push_back(i1);           tempFaces.push_back(i2);     tempFaces.push_back(i3);


        }
        faces = tempFaces;
    }


    if (tetra == true) {  // Using Tetrahedrons (uses tetgen)
        tetgenio in, out;
        in.numberofpoints = vertices.size();
        in.pointlist = new REAL[in.numberofpoints * 3];

        for (int i = 0; i < vertices.size(); ++i) {
            in.pointlist[i * 3 + 0] = vertices[i].pos.x;
            in.pointlist[i * 3 + 1] = vertices[i].pos.y;
            in.pointlist[i * 3 + 2] = vertices[i].pos.z;
        }

        in.numberoffacets = faces.size() / 3;
        in.facetlist = new tetgenio::facet[in.numberoffacets];
        in.facetmarkerlist = new int[in.numberoffacets];

        for (int i = 0; i < in.numberoffacets; ++i) {
            tetgenio::facet& f = in.facetlist[i];
            f.numberofpolygons = 1;
            f.polygonlist = new tetgenio::polygon[1];
            f.numberofholes = 0;
            f.holelist = nullptr;

            tetgenio::polygon& p = f.polygonlist[0];
            p.numberofvertices = 3;
            p.vertexlist = new int[3];
            p.vertexlist[0] = faces[i * 3 + 0];
            p.vertexlist[1] = faces[i * 3 + 1];
            p.vertexlist[2] = faces[i * 3 + 2];

            in.facetmarkerlist[i] = 0;
        }

        in.numberofregions = 1;
        in.regionlist = new REAL[4];
        in.regionlist[0] = cx;
        in.regionlist[1] = cy;
        in.regionlist[2] = cz;
        in.regionlist[3] = 1;

        tetgenbehavior behavior;
        behavior.parse_commandline((char*)"pa0.1Y");
        tetrahedralize(&behavior, &in, &out);

        vertices.resize(out.numberofpoints);
        for (int i = 0; i < out.numberofpoints; i++) {
            vertices[i].pos = glm::vec3(out.pointlist[i*3], out.pointlist[i*3 + 1], out.pointlist[i*3 + 2]);
            vertices[i].mass = mass;
        }

        model.nodes = vertices;

        for (int i = 0; i < out.numberoftetrahedra; ++i) {
            int a = out.tetrahedronlist[i * 4 + 0];
            int b = out.tetrahedronlist[i * 4 + 1];
            int c = out.tetrahedronlist[i * 4 + 2];
            int d = out.tetrahedronlist[i * 4 + 3];

            vector<int> points {a,b,c,d};

            Tetra t;
            vector<unsigned int> outT;
            for (auto& p : points) {
                if (float dist = glm::length(vertices[p].pos - center.pos); radius - dist > 0.001f) {
                    t.in = p;
                }
                else {
                    outT.push_back(p);
                }
            }
            bool surface = true;
            if (outT.size() < 3 || outT.size() == 4) {
                t.in = a;
                outT.clear();
                outT.push_back(b);
                outT.push_back(c);
                outT.push_back(d);
                surface = false;
            }
            t.out = outT;
            t.volume = calculateTetraVolume(t, model);
            t.surface = surface;

            auto addSpring = [&](int i, int j) {
                Spring s;
                s.A = i;
                s.B = j;
                s.restLength = glm::length(vertices[i].pos - vertices[j].pos);
                if (glm::length(vertices[i].pos - center.pos) < radius || glm::length(vertices[j].pos - center.pos) < radius) {
                    s.compliance = centerCompliance;
                }
                else {
                    s.compliance = compliance;
                }
                springs.push_back(s);
            };

            addSpring(a, b);
            addSpring(b, c);
            addSpring(c, a);
            addSpring(a, d);
            addSpring(b, d);
            addSpring(c, d);

            tetras.push_back(t);
        }
        model.tetras = tetras;

        delete[] in.trifacelist;
    }
    else {
        model.nodes = vertices;

        vector<AngledSpring> angledSprings;

        for (int i = 0; i < faces.size(); i += 3) {
            int A = faces[i];
            int B = faces[i + 1];
            int C = faces[i + 2];

            Spring s1;
            Spring s2;
            Spring s3;

            s1.A = A;
            s1.B = B;
            s1.restLength = glm::length(vertices[A].pos - vertices[B].pos);
            s1.compliance = compliance;

            s2.A = B;
            s2.B = C;
            s2.restLength = glm::length(vertices[B].pos - vertices[C].pos);
            s2.compliance = compliance;

            s3.A = C;
            s3.B = A;
            s3.restLength = glm::length(vertices[C].pos - vertices[A].pos);
            s3.compliance = compliance;

            springs.push_back(s1);
            springs.push_back(s2);
            springs.push_back(s3);
        }

        for (int i = 0; i < faces.size() - 3; i += 3) {
            int A = faces[i];
            int B = faces[i + 1];
            int C = faces[i + 2];

            for (int j = i + 3; j < faces.size(); j += 3) {
                int D = faces[j];
                int E = faces[j + 1];
                int F = faces[j + 2];

                vector<glm::vec2> edgesA;
                vector<glm::vec2> edgesB;

                edgesA.push_back(glm::vec2(A,B));
                edgesA.push_back(glm::vec2(B,C));
                edgesA.push_back(glm::vec2(C,A));

                edgesB.push_back(glm::vec2(D,E));
                edgesB.push_back(glm::vec2(E,F));
                edgesB.push_back(glm::vec2(F,D));

                glm::vec2 edge = getEdge(edgesA, edgesB);

                if (edge != glm::vec2(0,0)) {
                    set<int> triangleA = {A, B, C};
                    set<int> triangleB = {D, E, F};

                    int uniqueA = -1, uniqueB = -1;
                    for (int point : triangleA) {
                        if (edge.x == point) {
                            uniqueA = point;
                            break;
                        }
                        else if (edge.y == point) {
                            uniqueA = point;
                            break;
                        }
                    }
                    for (int point : triangleB) {
                        if (edge.x == point) {
                            uniqueB = point;
                            break;
                        }
                        else if (edge.y == point) {
                            uniqueB = point;
                            break;
                        }
                    }

                    AngledSpring s;
                    s.A = uniqueA;
                    s.B = uniqueB;
                    s.edge = edge;
                    s.compliance = angleStiffness;
                    glm::vec3 v1 = glm::cross((vertices[uniqueA].pos - vertices[edge.x].pos), (vertices[edge.y].pos - vertices[edge.x].pos));
                    glm::vec3 v2 = glm::cross((vertices[uniqueB].pos - vertices[edge.x].pos), (vertices[edge.y].pos - vertices[edge.x].pos));
                    s.restAngle = acos(calculateAngle(v1, v2));
                    angledSprings.push_back(s);
                }
            }
        }
        model.angledSprings = angledSprings;
    }

    model.faces = vector<unsigned int>(faces.begin(), faces.end());
    model.springs = springs;
    model.center = center;
    model.type = 0;
    model.distV = model.distH = model.distL = radius;
    model.subStep = subStep;
    model.bounciness = bounciness;
    model.color = color;
    model.wireframe = wireframe;
    model.update = update;
    model.volume = calculateVolume(model);
    model.compliance = volumeCompliance;
    model.tetra = tetra;
    createBB(model);
    createModelVBHTetra(model);

    computeVertexNormals(model);
    uploadModelToGPU(model);
    printf("Number of points %d Number of Faces %d Number of springs %d \n", (int)model.nodes.size(), (int)model.faces.size(), (int)model.springs.size());
    return model;
}

Model makeSoftSphere(float radius, int slices, int stacks, float subStep = 1.0f, float mass = 1.0f, float bounciness = 0.0f, float volumeCompliance = 0.0f,
                    float compliance = 0.0f, float centerCompliance = 0.0f, float angleStiffness = 0.0f, float cx = 0, float cy = 0, float cz = 0,
                    glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f), bool wireframe = false, bool update = false, bool tetra = false) {
    vector<Node> vertices;
    vector<int> faces;
    vector<Spring> springs;
    vector<Tetra> tetras;

    Model model;

    // Top vertex
    Node topNode;
    topNode.pos = glm::vec3(cx, cy + radius, cz);
    topNode.mass = mass;
    vertices.push_back(topNode);
    int topIndex = 0;

    // Generate middle stack vertices (excluding poles)
    for (int i = 1; i < stacks; ++i) {
        float phi = M_PI * i / stacks;
        float y = cos(phi);
        float r = sin(phi);

        for (int j = 0; j < slices; ++j) {
            float theta = 2 * M_PI * j / slices;
            float x = r * cos(theta);
            float z = r * sin(theta);
            Node node;
            node.pos = glm::vec3(x * radius + cx, y * radius + cy, z * radius + cz);
            node.mass = mass;
            vertices.push_back(node);
        }
    }

    // Bottom vertex
    Node bottomNode;
    bottomNode.pos = glm::vec3(cx, cy - radius, cz);
    bottomNode.mass = mass;
    vertices.push_back(bottomNode);
    int bottomIndex = (int)vertices.size() - 1;

    // index helper (1-based stack index because of shared top)
    auto idx = [&](int stack, int slice) {
        slice = ((slice % slices) + slices) % slices;
        return 1 + (stack - 1) * slices + slice;
    };

    // Faces Generation

    // Top cap
    for (int j = 0; j < slices; ++j) {
        faces.push_back(idx(1, j + 1));
        faces.push_back(idx(1, j));
        faces.push_back(topIndex);
    }

    // Middle
    for (int i = 1; i < stacks - 1; ++i) {
        for (int j = 0; j < slices; ++j) {
            int a = idx(i, j);
            int b = idx(i + 1, j);
            int c = idx(i, j + 1);
            int d = idx(i + 1, j + 1);

            faces.push_back(a);
            faces.push_back(c);
            faces.push_back(b);

            faces.push_back(b);
            faces.push_back(c);
            faces.push_back(d);
        }
    }

    // Bottom cap
    for (int j = 0; j < slices; ++j) {
        faces.push_back(idx(stacks - 1, j));
        faces.push_back(idx(stacks - 1, j + 1));
        faces.push_back(bottomIndex);
    }

    // Model Center
    Node center;
    center.pos = glm::vec3(cx, cy, cz);
    center.mass = mass;


    if (tetra == true) {  // Using Tetrahedrons (uses tetgen)
        tetgenio in, out;
        in.numberofpoints = vertices.size();
        in.pointlist = new REAL[in.numberofpoints * 3];

        for (int i = 0; i < vertices.size(); ++i) {
            in.pointlist[i * 3 + 0] = vertices[i].pos.x;
            in.pointlist[i * 3 + 1] = vertices[i].pos.y;
            in.pointlist[i * 3 + 2] = vertices[i].pos.z;
        }

        in.numberoffacets = faces.size() / 3;
        in.facetlist = new tetgenio::facet[in.numberoffacets];
        in.facetmarkerlist = new int[in.numberoffacets];

        for (int i = 0; i < in.numberoffacets; ++i) {
            tetgenio::facet& f = in.facetlist[i];
            f.numberofpolygons = 1;
            f.polygonlist = new tetgenio::polygon[1];
            f.numberofholes = 0;
            f.holelist = nullptr;

            tetgenio::polygon& p = f.polygonlist[0];
            p.numberofvertices = 3;
            p.vertexlist = new int[3];
            p.vertexlist[0] = faces[i * 3 + 0];
            p.vertexlist[1] = faces[i * 3 + 1];
            p.vertexlist[2] = faces[i * 3 + 2];

            in.facetmarkerlist[i] = 0;
        }

        in.numberofregions = 1;
        in.regionlist = new REAL[4];
        in.regionlist[0] = cx;
        in.regionlist[1] = cy;
        in.regionlist[2] = cz;
        in.regionlist[3] = 1;

        tetgenbehavior behavior;
        behavior.parse_commandline((char*)"pa0.1Y");
        tetrahedralize(&behavior, &in, &out);

        vertices.resize(out.numberofpoints);
        for (int i = 0; i < out.numberofpoints; i++) {
            vertices[i].pos = glm::vec3(out.pointlist[i*3], out.pointlist[i*3 + 1], out.pointlist[i*3 + 2]);
            vertices[i].mass = mass;
        }

        model.nodes = vertices;

        for (int i = 0; i < out.numberoftetrahedra; ++i) {
            int a = out.tetrahedronlist[i * 4 + 0];
            int b = out.tetrahedronlist[i * 4 + 1];
            int c = out.tetrahedronlist[i * 4 + 2];
            int d = out.tetrahedronlist[i * 4 + 3];

            vector<int> points {a,b,c,d};

            Tetra t;
            vector<unsigned int> outT;
            for (auto& p : points) {
                if (float dist = glm::length(vertices[p].pos - center.pos); radius - dist > 0.001f) {
                    t.in = p;
                }
                else {
                    outT.push_back(p);
                }
            }
            bool surface = true;
            if (outT.size() < 3 || outT.size() == 4) {
                t.in = a;
                outT.clear();
                outT.push_back(b);
                outT.push_back(c);
                outT.push_back(d);
                surface = false;
            }
            t.out = outT;
            t.volume = calculateTetraVolume(t, model);
            t.surface = surface;

            auto addSpring = [&](int i, int j) {
                Spring s;
                s.A = i;
                s.B = j;
                s.restLength = glm::length(vertices[i].pos - vertices[j].pos);
                if (glm::length(vertices[i].pos - center.pos) < radius || glm::length(vertices[j].pos - center.pos) < radius) {
                    s.compliance = centerCompliance;
                }
                else {
                    s.compliance = compliance;
                }
                springs.push_back(s);
            };

            addSpring(a, b);
            addSpring(b, c);
            addSpring(c, a);
            addSpring(a, d);
            addSpring(b, d);
            addSpring(c, d);

            tetras.push_back(t);
        }
        model.tetras = tetras;

        delete[] in.trifacelist;
    }
    else {
        model.nodes = vertices;

        vector<AngledSpring> angledSprings;

        for (int i = 0; i < faces.size(); i += 3) {
            int A = faces[i];
            int B = faces[i + 1];
            int C = faces[i + 2];

            Spring s1;
            Spring s2;
            Spring s3;

            s1.A = A;
            s1.B = B;
            s1.restLength = glm::length(vertices[A].pos - vertices[B].pos);
            s1.compliance = compliance;

            s2.A = B;
            s2.B = C;
            s2.restLength = glm::length(vertices[B].pos - vertices[C].pos);
            s2.compliance = compliance;

            s3.A = C;
            s3.B = A;
            s3.restLength = glm::length(vertices[C].pos - vertices[A].pos);
            s3.compliance = compliance;

            springs.push_back(s1);
            springs.push_back(s2);
            springs.push_back(s3);
        }

        for (int i = 0; i < faces.size() - 3; i += 3) {
            int A = faces[i];
            int B = faces[i + 1];
            int C = faces[i + 2];

            for (int j = i + 3; j < faces.size(); j += 3) {
                int D = faces[j];
                int E = faces[j + 1];
                int F = faces[j + 2];

                vector<glm::vec2> edgesA;
                vector<glm::vec2> edgesB;

                edgesA.push_back(glm::vec2(A,B));
                edgesA.push_back(glm::vec2(B,C));
                edgesA.push_back(glm::vec2(C,A));

                edgesB.push_back(glm::vec2(D,E));
                edgesB.push_back(glm::vec2(E,F));
                edgesB.push_back(glm::vec2(F,D));

                glm::vec2 edge = getEdge(edgesA, edgesB);

                if (edge != glm::vec2(0,0)) {
                    set<int> triangleA = {A, B, C};
                    set<int> triangleB = {D, E, F};

                    int uniqueA = -1, uniqueB = -1;
                    for (int point : triangleA) {
                        if (edge.x == point) {
                            uniqueA = point;
                            break;
                        }
                        else if (edge.y == point) {
                            uniqueA = point;
                            break;
                        }
                    }
                    for (int point : triangleB) {
                        if (edge.x == point) {
                            uniqueB = point;
                            break;
                        }
                        else if (edge.y == point) {
                            uniqueB = point;
                            break;
                        }
                    }

                    AngledSpring s;
                    s.A = uniqueA;
                    s.B = uniqueB;
                    s.edge = edge;
                    s.compliance = angleStiffness;
                    glm::vec3 v1 = glm::cross((vertices[uniqueA].pos - vertices[edge.x].pos), (vertices[edge.y].pos - vertices[edge.x].pos));
                    glm::vec3 v2 = glm::cross((vertices[uniqueB].pos - vertices[edge.x].pos), (vertices[edge.y].pos - vertices[edge.x].pos));
                    s.restAngle = acos(calculateAngle(v1, v2));
                    angledSprings.push_back(s);
                }
            }
        }
        model.angledSprings = angledSprings;
    }

    model.faces = vector<unsigned int>(faces.begin(), faces.end());
    model.springs = springs;
    model.center = center;
    model.type = 0;
    model.distV = model.distH = model.distL = radius;
    model.subStep = subStep;
    model.bounciness = bounciness;
    model.color = color;
    model.wireframe = wireframe;
    model.update = update;
    model.volume = calculateVolume(model);
    model.compliance = volumeCompliance;
    model.tetra = tetra;
    createBB(model);
    createModelVBHTetra(model);

    uploadModelToGPU(model);
    printf("Number of points %d Number of Faces %d Number of springs %d \n", (int)model.nodes.size(), (int)model.faces.size(), (int)model.springs.size());
    return model;
}

// Makes a Rigid Object with the shape of a Sphere
Model makeRigidSphere(float radius, int slices, int stacks, float mass = 1.0f, float bounciness = 0.0f,
                    float cx = 0, float cy = 0, float cz = 0,
                    glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f), bool wireframe = false, bool update = false) {
    vector<Node> vertices;
    vector<int> faces;

    for (int i = 0; i <= stacks; ++i) {
        float phi = M_PI * i / stacks;
        float y = cos(phi);
        float r = sin(phi);

        for (int j = 0; j <= slices; ++j) {
            float theta = 2 * M_PI * j / slices;
            float x = r * cos(theta);
            float z = r * sin(theta);
            Node node;
            node.pos = glm::vec3(x*radius+cx, y*radius+cy, z*radius+cz);
            vertices.push_back(node);
        }
    }


    for (int i = 0; i < stacks; ++i) {
        for (int j = 0; j < slices; ++j) {
            int first = i * (slices + 1) + j;
            int second = first + slices + 1;

            faces.push_back(first);
            faces.push_back(first + 1);
            faces.push_back(second);

            faces.push_back(second);
            faces.push_back(first + 1);
            faces.push_back(second + 1);

        }
    }

    Node center;
    center.pos = glm::vec3(cx, cy, cz);
    center.mass = mass;

    Model model;

    model.nodes = vertices;

    model.faces = vector<unsigned int>(faces.begin(), faces.end());

    model.type = 2;
    model.distV = radius, model.distH = radius, model.distL = radius;
    model.center = center;
    model.bounciness = bounciness;
    model.color = color;
    model.wireframe = wireframe;
    model.update = update;
    createBB(model);
    computeVertexNormals(model);
    uploadModelToGPU(model);
    return model;
}

// Makes a Soft Object with the shape of a Box
Model makeSoftBox(int slices, int stacks, float subStep, float mass = 1.0f, float bounciness = 0.0f, float volumeCompliance = 0.0f,
                float compliance = 0.0f, float centerCompliance = 0.0f , float angleStiffness = 0.0f, float cx = 0, float cy = 0, float cz = 0, float H = 1, float V = 1, float L = 1,
                glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f), bool wireframe = false, bool update = false, bool tetra = false) {
    vector<Node> vertices;
    vector<int> faces;
    vector<Spring> springs;
    std::unordered_map<glm::vec3, int, Vec3Hash, Vec3Equal> vertexMap;
    vector<Tetra> tetras;
    Model model;

    //set<glm::vec3, Vec3Less> positions;
    vector<glm::vec3> positions;
    vector<int> indexs;
    int index = 0;
    float x, y, z;
/*
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        z = -L;
        for (int j = 0; j <= slices; ++j) {
            x = -H + H*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }

        x = H;
        for (int j = 0; j <= slices; ++j) {
            z = -L + L*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }

        z = L;
        for (int j = 0; j <= slices; ++j) {
            z = L - L*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }

        x = -H;
        for (int j = 0; j <= slices; ++j) {
            z = L - L*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }
    }*/

    // First Face
    z = -L;
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            x = -H + H*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }
    }

    // Second Face
    x = H;
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            z = -L + L*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }
    }


    // Third Face
    z = L;
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            x = H - H*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }
    }

    // Fourth Face
    x = -H;
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            z = L - L*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }
    }

    // Fith Face
    y = -V;
    for (int i = 0; i <= stacks; ++i) {
        x = -H + H*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            z = -L + L*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }
    }

    // Sixth Face
    y = V;
    for (int i = 0; i <= stacks; ++i) {
        x = H - H*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            z = -L + L*2/slices*j;
            glm::vec3 vertex = glm::vec3(x + cx, y + cy, z + cz);
            if (vertexMap.find(vertex) == vertexMap.end()) {
                vertexMap[vertex] = index;
                positions.push_back(vertex);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[vertex]);
            }
        }
    }

    for (auto& p : positions) {
        Node node;
        node.pos = p;
        node.mass = mass;
        vertices.push_back(node);
    }

    int faceSize = (slices + 1) * (stacks + 1);
    for (int k = 0; k < 6; k++) {
        int offset = k * faceSize;
        for (int i = 0; i < stacks; ++i) {
            for (int j = 0; j < slices; ++j) {
                int a = offset + j + (slices + 1) * i;
                int b = offset + j + (slices + 1) * (i + 1);
                int c = a + 1;
                int d = b + 1;

                faces.push_back(indexs[a]);
                faces.push_back(indexs[b]);
                faces.push_back(indexs[c]);

                faces.push_back(indexs[b]);
                faces.push_back(indexs[d]);
                faces.push_back(indexs[c]);
            }
        }
    }

    Node center;
    //vector<Spring> springsC;
    center.pos = glm::vec3(cx, cy, cz);
    center.mass = mass;

    if (tetra) {
        // Tetrahedralize using Tetgen
        tetgenio in, out;
        in.numberofpoints = vertices.size();
        in.pointlist = new REAL[in.numberofpoints * 3];
        for (int i = 0; i < vertices.size(); ++i) {
            in.pointlist[i * 3 + 0] = vertices[i].pos.x;
            in.pointlist[i * 3 + 1] = vertices[i].pos.y;
            in.pointlist[i * 3 + 2] = vertices[i].pos.z;
        }

        in.numberoffacets = faces.size() / 3;
        in.facetlist = new tetgenio::facet[in.numberoffacets];
        in.facetmarkerlist = new int[in.numberoffacets];

        for (int i = 0; i < in.numberoffacets; ++i) {
            tetgenio::facet& f = in.facetlist[i];
            f.numberofpolygons = 1;
            f.polygonlist = new tetgenio::polygon[1];
            f.numberofholes = 0;
            f.holelist = nullptr;

            tetgenio::polygon& p = f.polygonlist[0];
            p.numberofvertices = 3;
            p.vertexlist = new int[3];
            p.vertexlist[0] = faces[i * 3 + 0];
            p.vertexlist[1] = faces[i * 3 + 1];
            p.vertexlist[2] = faces[i * 3 + 2];

            in.facetmarkerlist[i] = 0;
        }

        in.numberofregions = 1;
        in.regionlist = new REAL[4];
        in.regionlist[0] = cx;
        in.regionlist[1] = cy;
        in.regionlist[2] = cz;
        in.regionlist[3] = 1;

        tetgenbehavior behavior;
        behavior.parse_commandline((char*)"pa0.1Y");
        tetrahedralize(&behavior, &in, &out);

        // Overwrite vertices
        vertices.resize(out.numberofpoints);
        for (int i = 0; i < out.numberofpoints; ++i) {
            vertices[i].pos = glm::vec3(out.pointlist[i * 3], out.pointlist[i * 3 + 1], out.pointlist[i * 3 + 2]);
            vertices[i].mass = mass;
        }

        model.nodes = vertices;

        for (int i = 0; i < out.numberoftetrahedra; ++i) {
            int a = out.tetrahedronlist[i * 4 + 0];
            int b = out.tetrahedronlist[i * 4 + 1];
            int c = out.tetrahedronlist[i * 4 + 2];
            int d = out.tetrahedronlist[i * 4 + 3];

            vector<int> points{a, b, c, d};

            Tetra t;
            vector<unsigned int> outT;
            bool surface = true;
            for (auto& p : points) {
                const glm::vec3& pos = vertices[p].pos;

                bool insideX = (pos.x > cx - H + EPSILON) && (pos.x < cx + H - EPSILON);
                bool insideY = (pos.y > cy - V + EPSILON) && (pos.y < cy + V - EPSILON);
                bool insideZ = (pos.z > cz - L + EPSILON) && (pos.z < cz + L - EPSILON);

                if (insideX && insideY && insideZ) {
                    t.in = p;
                } else {
                    outT.push_back(p);
                }
            }
            if (outT.size() < 3 || outT.size() == 4) {
                t.in = a;
                outT.clear();
                outT.push_back(b);
                outT.push_back(c);
                outT.push_back(d);
                surface = false;
            }
            t.out = outT;
            t.volume = calculateTetraVolume(t, model);
            t.surface = surface;

            auto addSpring = [&](int i, int j) {
                Spring s;
                s.A = i;
                s.B = j;
                s.restLength = glm::length(vertices[i].pos - vertices[j].pos);
                s.compliance = (glm::length(vertices[i].pos - center.pos) < std::max(H, std::max(V, L))) ||
                                glm::length(vertices[j].pos - center.pos) < std::max(H, std::max(V, L))
                               ? centerCompliance : compliance;
                springs.push_back(s);
            };

            addSpring(a, b);
            addSpring(b, c);
            addSpring(c, a);
            addSpring(a, d);
            addSpring(b, d);
            addSpring(c, d);

            tetras.push_back(t);
        }

        model.tetras = tetras;
    }
    else {
        model.nodes = vertices;
        for (int i = 0; i < faces.size(); i += 3) {
            int A = faces[i];
            int B = faces[i + 1];
            int C = faces[i + 2];

            Spring s1;
            Spring s2;
            Spring s3;

            s1.A = A;
            s1.B = B;
            s1.restLength = glm::length(vertices[A].pos - vertices[B].pos);
            s1.compliance = compliance;

            s2.A = B;
            s2.B = C;
            s2.restLength = glm::length(vertices[B].pos - vertices[C].pos);
            s2.compliance = compliance;

            s3.A = C;
            s3.B = A;
            s3.restLength = glm::length(vertices[C].pos - vertices[A].pos);
            s3.compliance = compliance;

            springs.push_back(s1);
            springs.push_back(s2);
            springs.push_back(s3);
        }

        vector<AngledSpring> angledSprings;

        for (int i = 0; i < faces.size() - 3; i += 3) {
            int A = faces[i];
            int B = faces[i + 1];
            int C = faces[i + 2];

            for (int j = i + 3; j < faces.size(); j += 3) {
                int D = faces[i + 3];
                int E = faces[i + 4];
                int F = faces[i + 5];

                vector<glm::vec2> edgesA;
                vector<glm::vec2> edgesB;

                edgesA.push_back(glm::vec2(A,B));
                edgesA.push_back(glm::vec2(B,C));
                edgesA.push_back(glm::vec2(C,A));

                edgesB.push_back(glm::vec2(D,E));
                edgesB.push_back(glm::vec2(E,F));
                edgesB.push_back(glm::vec2(F,D));

                glm::vec2 edge = getEdge(edgesA, edgesB);

                if (edge != glm::vec2(0,0)) {
                    set<int> triangleA = {A, B, C};
                    set<int> triangleB = {D, E, F};

                    int uniqueA = -1, uniqueB = -1;
                    for (int point : triangleA) {
                        if (edge.x == point) {
                            uniqueA = point;
                            break;
                        }
                        else if (edge.y == point) {
                            uniqueA = point;
                            break;
                        }
                    }
                    for (int point : triangleB) {
                        if (edge.x == point) {
                            uniqueB = point;
                            break;
                        }
                        else if (edge.y == point) {
                            uniqueB = point;
                            break;
                        }
                    }

                    AngledSpring s;
                    s.A = uniqueA;
                    s.B = uniqueB;
                    s.edge = edge;
                    s.compliance = angleStiffness;
                    glm::vec3 v1 = glm::cross((vertices[uniqueA].pos - vertices[edge.x].pos), (vertices[edge.y].pos - vertices[edge.x].pos));
                    glm::vec3 v2 = glm::cross((vertices[uniqueB].pos - vertices[edge.x].pos), (vertices[edge.y].pos - vertices[edge.x].pos));
                    s.restAngle = acos(calculateAngle(v1, v2));
                    angledSprings.push_back(s);
                }
            }
        }
        model.angledSprings = angledSprings;
    }

    //std::mt19937 gen(std::random_device{}());
    //std::uniform_int_distribution<> randVert(0, (int)vertices.size() - 1);
    //vertices.push_back(center);

    /*
    int addedToCenter = 0;
    while (addedToCenter < 10) {
        int A = randVert(gen);
        Spring s;
        s.A = 1;
        s.B = A;
        s.compliance = compliance;
        s.restLength = glm::length(vertices[A].pos - center.pos);
        springsC.push_back(s);
        ++addedToCenter;
    }*/


    // Build final model
    model.faces = vector<unsigned int>(faces.begin(), faces.end());
    model.springs = springs;
    model.volume = calculateVolume(model);
    model.compliance = volumeCompliance;

    model.center = center;
    model.type = 0;
    model.distV = V, model.distH = H, model.distL = L;
    model.center = center;
    model.bounciness = bounciness;
    model.color = color;
    model.wireframe = wireframe;
    model.update = update;
    model.subStep = subStep;
    createBB(model);
    createModelVBHTetra(model);
    computeVertexNormals(model);
    uploadModelToGPU(model);
    printf("Number of points %d Number of Faces %d Number of springs %d \n", (int)model.nodes.size(), (int)model.faces.size(), (int)model.springs.size());
    return model;
}

// Makes a Rigid Object with the shape of a Box
Model makeRigidBox(int slices, int stacks, bool volumeBased = false, float mass = 1.0f, float bounciness = 0.0f,
                float cx = 0, float cy = 0, float cz = 0, float H = 1, float V = 1, float L = 1,
                glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f), bool wireframe = false, bool update = false) {
    vector<Node> vertices;
    vector<int> faces;

    auto addFace = [&](float ox, float oy, float oz, float dx1, float dy1, float dz1, float dx2, float dy2, float dz2) {
        int start = (int)vertices.size();
        for (int i = 0; i <= stacks; ++i) {
            float v = (float)i / stacks;
            for (int j = 0; j <= slices; ++j) {
                float u = (float)j / slices;
                Node node;
                node.pos = glm::vec3(ox + dx1 * u + dx2 * v, oy + dy1 * u + dy2 * v, oz + dz1 * u + dz2 * v);
                vertices.push_back(node);
            }
        }
        for (int i = 0; i < stacks; ++i) {
            for (int j = 0; j < slices; ++j) {
                int first = start + i * (slices + 1) + j;
                int second = first + slices + 1;

                faces.push_back(first);
                faces.push_back(second);
                faces.push_back(first + 1);

                faces.push_back(second);
                faces.push_back(second + 1);
                faces.push_back(first + 1);
            }
        }
    };

    // Add all 6 cube faces
    addFace(cx - H, cy + V, cz + L, 2 * H, 0, 0, 0, -2 * V, 0);  // Front (+Z)
    addFace(cx + H, cy + V, cz - L, -2 * H, 0, 0, 0, -2 * V, 0); // Back (-Z)
    addFace(cx - H, cy + V, cz - L, 2 * H, 0, 0, 0, 0, 2 * L);   // Top (+Y)
    addFace(cx - H, cy - V, cz + L, 2 * H, 0, 0, 0, 0, -2 * L);  // Bottom (-Y)
    addFace(cx + H, cy + V, cz + L, 0, 0, -2 * L, 0, -2 * V, 0); // Right (+X)
    addFace(cx - H, cy + V, cz - L, 0, 0, 2 * L, 0, -2 * V, 0);  // Left (-X)

    // Central node and springs
    Node center;
    center.pos = glm::vec3(cx, cy, cz);
    center.mass = mass;

    // Build final model
    Model model;
    model.nodes = vertices;
    model.faces = vector<unsigned int>(faces.begin(), faces.end());
    model.center = center;
    model.type = 1;
    model.distV = V, model.distH = H, model.distL = L;
    model.center = center;
    model.bounciness = bounciness;
    model.color = color;
    model.wireframe = wireframe;
    model.update = update;
    createBB(model);
    computeVertexNormals(model);
    uploadModelToGPU(model);
    return model;
}


void uploadModelToGPU(Model &model)
{
    if (model.nodes.empty() || model.faces.empty()) return;

    // Prepare vertex buffer
    std::vector<Vertex> verts(model.nodes.size());
    for (size_t i = 0; i < model.nodes.size(); ++i) {
        verts[i].pos    = model.nodes[i].pos;
        verts[i].normal = model.nodes[i].lighNormal;
    }

    GLModel &g = model.glModel;

    // Generate VAO
    glGenVertexArrays(1, &g.vao);
    glBindVertexArray(g.vao);

    // VBO for vertex data
    glGenBuffers(1, &g.vboVertices);
    glBindBuffer(GL_ARRAY_BUFFER, g.vboVertices);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(Vertex), verts.data(), GL_DYNAMIC_DRAW);

    // VBO for indices
    glGenBuffers(1, &g.vboIndices);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g.vboIndices);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, model.faces.size() * sizeof(unsigned int), model.faces.data(), GL_STATIC_DRAW);

    // Vertex attributes
    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

    glEnableVertexAttribArray(1); // normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));

    // Unbind VAO (optional for safety)
    glBindVertexArray(0);

    g.indexCount = (GLsizei)model.faces.size();
}


//--------------------------------------------------------------
//  Update only positions inside an interleaved VBO – NEW
//--------------------------------------------------------------
void updateGPUpositionsAndNormals(Model &model)
{
    GLsizei stride = sizeof(Vertex);
    glBindBuffer(GL_ARRAY_BUFFER, model.glModel.vboVertices);

    for (size_t i = 0; i < model.nodes.size(); ++i) {
        GLsizeiptr baseOffset = static_cast<GLsizeiptr>(i * stride);
        glBufferSubData(GL_ARRAY_BUFFER, baseOffset, sizeof(glm::vec3), &model.nodes[i].pos);
        glBufferSubData(GL_ARRAY_BUFFER, baseOffset + sizeof(glm::vec3), sizeof(glm::vec3), &model.nodes[i].lighNormal);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}



bool isPointInTriangle(const glm::vec3& pt, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));

    // Barycentric technique using cross products
    glm::vec3 edge0 = b - a;
    glm::vec3 vp0 = pt - a;
    glm::vec3 c0 = glm::cross(edge0, vp0);
    if (glm::dot(n, c0) < 0) return false;

    glm::vec3 edge1 = c - b;
    glm::vec3 vp1 = pt - b;
    glm::vec3 c1 = glm::cross(edge1, vp1);
    if (glm::dot(n, c1) < 0) return false;

    glm::vec3 edge2 = a - c;
    glm::vec3 vp2 = pt - c;
    glm::vec3 c2 = glm::cross(edge2, vp2);
    if (glm::dot(n, c2) < 0) return false;

    return true;
}

bool rayIntersectsTriangle(
    const glm::vec3& orig, const glm::vec3& dir,
    const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
    float* outT = nullptr)
{
    glm::vec3 edge1 = v1 - v0;
    glm::vec3 edge2 = v2 - v0;

    glm::vec3 h = glm::cross(dir, edge2);
    float a = glm::dot(edge1, h);
    //if (a > -EPSILON)
        //return false;  // Backface or parallel — cull

    if (fabs(a) < EPSILON)
        return false;  // Backface or parallel — cull


    float f = 1.0f / a;
    glm::vec3 s = orig - v0;
    float u = f * glm::dot(s, h);
    if (u < 0.0f || u > 1.0f)
        return false;

    glm::vec3 q = glm::cross(s, edge1);
    float v = f * glm::dot(dir, q);
    if (v < 0.0f || u + v > 1.0f)
        return false;

    // At this stage, ray intersects triangle
    float t = f * glm::dot(edge2, q);
    if (t > EPSILON) {
        if (outT) *outT = t; // Optional: output distance to intersection
        return true;
    }

    return false;
}

bool rayIntersectsBB (glm::vec3 p, glm::vec3 ray, BB bb) {
    float tminX, tminY, tminZ;
    float tmaxX, tmaxY, tmaxZ;
    float tTemp, tMax, tMin;

    float invX = fabs(ray.x) > EPSILON ? 1.0f / ray.x : 1e8f; // Or std::numeric_limits<float>::max()
    tminX = (bb.minX - p.x) * invX;
    tmaxX = (bb.maxX - p.x) * invX;


    if (tminX > tmaxX) {
        tTemp = tmaxX;
        tmaxX = tminX;
        tminX = tTemp;
    }

    float invY = fabs(ray.y) > EPSILON ? 1.0f / ray.y : 1e8f; // Or std::numeric_limits<float>::max()
    tminY = (bb.minY - p.y) * invY;
    tmaxY = (bb.maxY - p.y) * invY;


    if (tminY > tmaxY) {
        tTemp = tmaxY;
        tmaxY = tminY;
        tminY = tTemp;
    }

    float invZ = fabs(ray.z) > EPSILON ? 1.0f / ray.z : 1e8f; // Or std::numeric_limits<float>::max()
    tminZ = (bb.minZ - p.z) * invZ;
    tmaxZ = (bb.maxZ - p.z) * invZ;


    if (tminZ > tmaxZ) {
        tTemp = tmaxZ;
        tmaxZ = tminZ;
        tminZ = tTemp;
    }

    tMin = max(max(tminX, tminY),tminZ);
    tMax = min(min(tmaxX, tmaxY),tmaxZ);

    if (tMin > tMax) {
        return false;
    }
    return true;
}

float distanceToPlaneAlongDirection(
    const glm::vec3& point,  // Ray origin
    const glm::vec3& dir,    // Ray direction (should be normalized)
    const glm::vec3& p0,     // A point on the plane
    const glm::vec3& n)      // Plane normal (assumed normalized)
{
    float denom = glm::dot(n, dir);
    if (abs(denom) < 1e-6f) return -1; // Parallel

    float t = glm::dot(p0 - point, n) / denom;
    return t;
}

glm::vec3 farthestPointInDirectionPredicted(const Tetra &tetra, const glm::vec3 &dir, Model &model, glm::vec3 center) {
    float maxDot = -std::numeric_limits<float>::infinity();
    glm::vec3 bestPoint;

    auto check = [&](int idx) {
        glm::vec3 p = model.nodes[idx].predicted_pos;
        float d = glm::dot(p - center, dir);
        if (d > maxDot) {
            maxDot = d;
            bestPoint = p;
        }
    };

    check(tetra.in);
    for (int o : tetra.out) check(o);

    return bestPoint;
}

glm::vec3 farthestPointInDirection(const Tetra &tetra, const glm::vec3 &dir, Model &model, glm::vec3 center) {
    float maxDot = -std::numeric_limits<float>::infinity();
    glm::vec3 bestPoint;

    auto check = [&](int idx) {
        glm::vec3 p = model.nodes[idx].pos;
        float d = glm::dot(p - center, dir);
        if (d > maxDot) {
            maxDot = d;
            bestPoint = p;
        }
    };

    check(tetra.in);
    for (int o : tetra.out) check(o);

    return bestPoint;
}


glm::vec3 Support(const Tetra &tetra1, const Tetra &tetra2, const glm::vec3 &dir, Model &m1, Model &m2, glm::vec3 c1, glm::vec3 c2) {
    glm::vec3 p1 = farthestPointInDirectionPredicted(tetra1, dir, m1, c1);
    glm::vec3 p2 = farthestPointInDirection(tetra2, dir, m2, c2);
    return p1 - p2;
}

bool lineCase(vector<glm::vec3> &simplex, glm::vec3 &dir) {
    glm::vec3 A = simplex[0];
    glm::vec3 B = simplex[1];

    glm::vec3 AB = B - A;
    glm::vec3 AO = ORIGIN - A;

    if (glm::dot(AB, AO) > 0) {
        dir = glm::cross(glm::cross(AB,AO),AB);
    }
    else {
        simplex.clear();
        simplex.push_back(A);
        dir = AO;
    }
    return false;
}

bool triangleCase(vector<glm::vec3> &simplex, glm::vec3 &dir) {
    glm::vec3 A = simplex[0];
    glm::vec3 B = simplex[1];
    glm::vec3 C = simplex[2];

    glm::vec3 AB = B - A;
    glm::vec3 AC = C - A;
    glm::vec3 AO = ORIGIN - A;

    glm::vec3 ABC = glm::cross(AB, AC);

    if (glm::dot(glm::cross(ABC, AC), AO) > 0) {
        if (glm::dot(AC, AO) > 0) {
            simplex.clear();
            simplex.push_back(A);
            simplex.push_back(C);
            dir = glm::cross(glm::cross(AC,AO), AC);
        }
        else {
            simplex.clear();
            simplex.push_back(A);
            simplex.push_back(B);
            return lineCase(simplex, dir);
        }
    }
    else {
        if (glm::dot(AB, ABC) > 0) {
            simplex.clear();
            simplex.push_back(A);
            simplex.push_back(B);
            return lineCase(simplex, dir);
        }
        else {
            if (glm::dot(ABC, AO) > 0) {
                dir = ABC;
            }
            else {
                simplex.clear();
                simplex.push_back(A);
                simplex.push_back(C);
                simplex.push_back(B);
                dir = -ABC;
            }
        }
    }
    return false;
}

bool tetraCase(vector<glm::vec3> &simplex, glm::vec3 &dir) {
    glm::vec3 A = simplex[0];
    glm::vec3 B = simplex[1];
    glm::vec3 C = simplex[2];
    glm::vec3 D = simplex[3];

    glm::vec3 AB = B - A;
    glm::vec3 AC = C - A;
    glm::vec3 AD = D - A;
    glm::vec3 AO = ORIGIN - A;

    glm::vec3 ABC = glm::cross(AB, AC);
    glm::vec3 ACD = glm::cross(AC, AD);
    glm::vec3 ADB = glm::cross(AD, AB);

    if (glm::dot(ABC, AO) > 0) {
        simplex.clear();
        simplex.push_back(A);
        simplex.push_back(B);
        simplex.push_back(C);
        return triangleCase(simplex, dir);
    }
    if (glm::dot(ACD, AO) > 0) {
        simplex.clear();
        simplex.push_back(A);
        simplex.push_back(C);
        simplex.push_back(D);
        return triangleCase(simplex, dir);
    }
    if (glm::dot(ADB, AO) > 0) {
        simplex.clear();
        simplex.push_back(A);
        simplex.push_back(D);
        simplex.push_back(B);
        return triangleCase(simplex, dir);
    }
    return true;
}

bool handleSimplex(vector<glm::vec3> &simplex, glm::vec3 &dir) {
    if (simplex.size() == 2) {
        return lineCase(simplex, dir);
    }
    if (simplex.size() == 3) {
        return triangleCase(simplex, dir);
    }
    return tetraCase(simplex, dir);
}

bool GJK (Tetra &tetra1, Tetra &tetra2, Model &m1, Model &m2) {
    glm::vec3 c1 = (m1.nodes[tetra1.in].predicted_pos + m1.nodes[tetra1.out[0]].predicted_pos + m1.nodes[tetra1.out[1]].predicted_pos + m1.nodes[tetra1.out[2]].predicted_pos) * 0.25f;
    glm::vec3 c2 = (m2.nodes[tetra2.in].pos + m2.nodes[tetra2.out[0]].pos + m2.nodes[tetra2.out[1]].pos + m2.nodes[tetra2.out[2]].pos) * 0.25f;
    glm::vec3 d = glm::normalize(c1 - c2);
    vector<glm::vec3> simplex;
    simplex.push_back(Support(tetra1, tetra2, d, m1, m2, c1, c2));

    d = ORIGIN - simplex[0];
    int iteration = 0;
    const int maxIterations = 30;
    while (iteration++ < maxIterations) {
        glm::vec3 A = Support(tetra1, tetra2, d, m1, m2, c1, c2);

        if (glm::dot(A, d) < 0) {
            return false;
        }
        simplex.push_back(A);
        if (handleSimplex(simplex, d)) {
            return true;
        }
    }
    return false;
}

bool pointInTetra(const glm::vec3 &p, const Tetra &tetra, const Model &model) {
    glm::vec3 a = model.nodes[tetra.in].pos;
    glm::vec3 b = model.nodes[tetra.out[0]].pos;
    glm::vec3 c = model.nodes[tetra.out[1]].pos;
    glm::vec3 d = model.nodes[tetra.out[2]].pos;

    float v0 = glm::dot(glm::cross(b - a, c - a), d - a);
    float v1 = glm::dot(glm::cross(b - p, c - p), d - p);
    float v2 = glm::dot(glm::cross(a - p, c - p), d - p);
    float v3 = glm::dot(glm::cross(a - p, b - p), d - p);
    float v4 = glm::dot(glm::cross(a - p, b - p), c - p);

    return (v0 > 0 && v1 > 0 && v2 > 0 && v3 > 0 && v4 > 0) ||
           (v0 < 0 && v1 < 0 && v2 < 0 && v3 < 0 && v4 < 0);
}


bool tetraIntersect(const Tetra &a, const Tetra &b, const Model &m1, const Model &m2) {
    // 1. Check if any point of A is inside B
    for (unsigned int ai : {static_cast<unsigned int>(a.in), a.out[0], a.out[1], a.out[2]}) {
        if (pointInTetra(m1.nodes[ai].predicted_pos, b, m2)) return true;
    }
    // 2. Check if any point of B is inside A
    for (unsigned int bi : {static_cast<unsigned int>(b.in), b.out[0], b.out[1], b.out[2]}){
        if (pointInTetra(m2.nodes[bi].pos, a, m1)) return true;
    }
    // 3. (Optional) Check triangle-triangle intersection for all face pairs
    return false;
}


float distanceTetra (Tetra &tetra1, Tetra &tetra2, Model &m1, Model &m2) {
    float minDist = numeric_limits<float>::infinity();
    for (int o1 : tetra1.out) {
        for (int o2 : tetra2.out) {
            float len = glm::length(m1.nodes[o1].predicted_pos - m2.nodes[o2].pos);
            if (len < minDist) {
                minDist = len;
            }
        }
    }
    return minDist;
}

int BVHrayCasting (Tetra &tetra, BVH &bvh, Model &m1, Model &m2) {
    if (bvh.left == -1 && bvh.right == -1) {
        if (tetraIntersect(tetra, m2.tetras[bvh.tetra], m1, m2)) {
            return bvh.tetra;
        }

        return -1;
    }
    else {
        BVH &left = m2.bvhs[bvh.left];
        BVH &right = m2.bvhs[bvh.right];

        bool insideLeft = intersectBB(tetra.bb, left.bb);
        bool insideRight = intersectBB(tetra.bb, right.bb);

        int result = -1;

        if (insideLeft) {
            result = BVHrayCasting(tetra, left, m1, m2);
        }

        if (insideRight) {
            if (result != -1) {
                int temp = BVHrayCasting(tetra, right, m1, m2);
                if (temp != -1) {
                    if (distanceTetra(tetra, m2.tetras[temp], m1, m2) < distanceTetra(tetra, m2.tetras[result], m1, m2)) {
                        result = temp;
                    }
                }
            }
            else {
                result = BVHrayCasting(tetra, right, m1, m2);
            }
        }
        return result;
    }
}


bool SameSide(glm::vec3 v1,glm::vec3 v2,glm::vec3 v3,glm::vec3 v4,glm::vec3 p)
{
    glm::vec3 normal = glm::cross(v2 - v1, v3 - v1);
    float dotV4 = glm::dot(normal, v4 - v1);
    float dotP = glm::dot(normal, p - v1);
    return (dotV4 <= 0 && dotP <= 0) || (dotV4 >= 0 && dotP >= 0);
}

bool PointInTetrahedron(glm::vec3 v1,glm::vec3 v2,glm::vec3 v3,glm::vec3 v4,glm::vec3 p)
{
    return SameSide(v1, v2, v3, v4, p) &&
           SameSide(v2, v3, v4, v1, p) &&
           SameSide(v3, v4, v1, v2, p) &&
           SameSide(v4, v1, v2, v3, p);
}

//------------------------------------------------------------------------------
//  HARD-body particle–model collision (minimal rewrite, same names / style)
//------------------------------------------------------------------------------
void handleCollisionPM(Node& node, Model& model) {
    // Use predicted position during constraint solve
    glm::vec3& pos = node.predicted_pos;
    glm::vec3& previous_pos = node.pos;

    if (pos.x > model.bb.minX && pos.x < model.bb.maxX && pos.y > model.bb.minY && pos.y < model.bb.maxY && pos.z > model.bb.minZ && pos.z < model.bb.maxZ) {
        if (model.type == 1) {
            // -------- A. Axis-Aligned Box --------------
            glm::vec3 c = model.center.pos;
            float H = model.distH, V = model.distV, L = model.distL;

            if (pos.x > c.x - H && pos.x < c.x + H &&
                pos.y > c.y - V && pos.y < c.y + V &&
                pos.z > c.z - L && pos.z < c.z + L) {


                float dx = std::min(c.x + H - previous_pos.x, previous_pos.x - (c.x - H));
                float dy = std::min(c.y + V - previous_pos.y, previous_pos.y - (c.y - V));
                float dz = std::min(c.z + L - previous_pos.z, previous_pos.z - (c.z - L));

                glm::vec3 n(0.0f);
                float pen = dx;
                n = glm::vec3((pos.x > c.x ? c.x + H : c.x - H), pos.y, pos.z);
                glm::vec3 normal = glm::vec3((pos.x > c.x ? 1.0f : -1.0f), 0.0f, 0.0f);
                if (dy < pen) { pen = dy; n = glm::vec3(pos.x, (pos.y > c.y ? c.y + V : c.y - V), pos.z); normal = glm::vec3(0.0f, (pos.y > c.y ? 1.0f : -1.0f), 0.0f);}
                if (dz < pen) { pen = dz; n = glm::vec3(pos.x, pos.y, (pos.z > c.z ? c.z + L : c.z - L)); normal = glm::vec3(0.0f, 0.0f, (pos.z > c.z ? 1.0f : -1.0f));}

                node.predicted_pos = n;
                node.hasCollided = true;
                node.normal = normal;

                /*
                float vn = glm::dot(node.vel, n);
                if (vn < 0.0f) {
                    glm::vec3 tangentVel = node.vel - vn * n;
                    node.vel -= friction * tangentVel;
                    node.vel -= (1.0f + model.bounciness) * vn * n;
                } */
                }
            return;
        }

        if (model.type == 2) {
            // -------- B. Sphere --------------
            glm::vec3 delta = pos - model.center.pos;
            float r = model.distH;
            float d = glm::length(delta);

            if (d <= r) {
                glm::vec3 normal = glm::normalize(delta);

                node.predicted_pos = model.center.pos + normal * r;
                node.hasCollided = true;
                node.normal = normal;
            }
            return;
        }
    }
}

void handleCollisionSS(Model &m1, Model &m2) {
    for (Tetra& t : m1.tetras) {
        if (intersectBB(t.bb, m2.bvhs.back().bb) && t.surface) {
            int tetra = BVHrayCasting(t, m2.bvhs.back(), m1, m2);
            if (tetra != -1) {
                glm::vec3 p1 = m2.nodes[m2.tetras[tetra].out[0]].pos;
                glm::vec3 n1 = glm::normalize(glm::cross(m2.nodes[m2.tetras[tetra].out[1]].pos - p1, m2.nodes[m2.tetras[tetra].out[2]].pos - p1));

                if (glm::dot(n1, m1.nodes[t.in].pos - m2.nodes[m2.tetras[tetra].in].pos) < 0) n1 = -n1;

                float tMass = (m2.nodes[m2.tetras[tetra].out[0]].mass + m2.nodes[m2.tetras[tetra].out[1]].mass +m2.nodes[m2.tetras[tetra].out[2]].mass)/3.0f;

                for (int o : t.out) {
                    glm::vec3 p = m1.nodes[o].predicted_pos;
                    if (glm::dot(p - p1, n1) < 0) {
                        float dist = distanceToPlaneAlongDirection(p, n1, p1, n1);
                        float totalMass = m1.nodes[o].mass + tMass;
                        float ratio = tMass / totalMass;
                        m1.nodes[o].hasCollided = true;
                        m1.nodes[o].normal = n1;
                        m1.nodes[o].predicted_pos += n1 * ratio * dist;
                    }
                }

                glm::vec3 p2 = m1.nodes[t.out[0]].predicted_pos;
                glm::vec3 n2 = glm::normalize(glm::cross(m1.nodes[t.out[1]].predicted_pos - p2, m1.nodes[t.out[2]].predicted_pos - p2));

                for (int o : m2.tetras[tetra].out) {
                    glm::vec3 p = m2.nodes[o].pos;
                    if (glm::dot(p - p2, n2) < 0) {
                        float dist = distanceToPlaneAlongDirection(p, -n1, p2, n2);
                        m2.nodes[o].predicted_pos += -n1 * dist;
                    }
                }

            }
        }
    }
    updateBVH(m2);
}

void handleCollision(Model &model) {
    for (Model& m : models) {
        if (&m != &model) {
            if (m.type == 0) {
                handleCollisionSS(model, m);
                updateBVH(model);
            }
            else {
                for (Node& n : model.nodes) {
                    handleCollisionPM(n, m);
                }
            }
        }
    }
}


void resolveCubeCollision(Node& node, glm::vec3 v, Model& model, bool useAnisotropicFriction) {
    float cx = model.center.pos.x;
    float cy = model.center.pos.y;
    float cz = model.center.pos.z;
    float H  = model.distH;           // half-width  (X)
    float V  = model.distV;           // half-height (Y)
    float L  = model.distL;           // half-length (Z)

    // -------- 1. is the point inside?  ------------------------------------
    if ( v.x > cx - H && v.x < cx + H &&
         v.y > cy - V && v.y < cy + V &&
         v.z > cz - L && v.z < cz + L )
    {
        // -------- 2. find smallest overlap & its face normal -------------
        float dx = std::min(cx + H - v.x, v.x - (cx - H));
        float dy = std::min(cy + V - v.y, v.y - (cy - V));
        float dz = std::min(cz + L - v.z, v.z - (cz - L));

        glm::vec3 n(0.0f);            // normal pointing out of the cube
        float     pen = dx;           // penetration depth along that normal

        n = glm::vec3((v.x > cx ? 1.0f : -1.0f), 0.0f, 0.0f);
        if (dy < pen) { pen = dy; n = glm::vec3(0.0f, (v.y > cy ? 1.0f : -1.0f), 0.0f); }
        if (dz < pen) { pen = dz; n = glm::vec3(0.0f, 0.0f, (v.z > cz ? 1.0f : -1.0f)); }

        // -------- 3. push the node out & reflect its velocity ------------
        node.pos += n * pen;          // positional correction
        float vn = glm::dot(node.vel, n);
        if (vn < 0.0f) {
            glm::vec3 tangentVel = node.vel - vn * n;  // Remove normal component
            node.vel -= friction * tangentVel;  // Apply friction

            // Bounce (normal component)
            node.vel -= (1.0f + model.bounciness) * vn * n;
        }
    }
}

void handleCollisionBM(Node& node, float CH, float CV, float CL, Model& model) {
    if (model.type == 1) {
        vector<glm::vec3> verts;
        verts.push_back(node.pos+glm::vec3(0.0f, CV, 0.0f));
        verts.push_back(node.pos-glm::vec3(0.0f, CV, 0.0f));
        verts.push_back(node.pos+glm::vec3(CH, 0.0f, 0.0f));
        verts.push_back(node.pos-glm::vec3(CH, 0.0f, 0.0f));
        verts.push_back(node.pos+glm::vec3(0.0f, 0.0f, CL));
        verts.push_back(node.pos-glm::vec3(0.0f, 0.0f, CL));

        for (auto& v : verts) {
            resolveCubeCollision(node, v, model, true);
        }
        return;
    }

    if (model.type == 2) {
        vector<glm::vec3> verts;
        verts.push_back(node.pos+glm::vec3(CH, CV, -CL));
        verts.push_back(node.pos+glm::vec3(CH, CV, CL));
        verts.push_back(node.pos+glm::vec3(CH, -CV, -CL));
        verts.push_back(node.pos+glm::vec3(CH, -CV, CL));
        verts.push_back(node.pos+glm::vec3(-CH, CV, -CL));
        verts.push_back(node.pos+glm::vec3(-CH, CV, CL));
        verts.push_back(node.pos+glm::vec3(-CH, -CV, -CL));
        verts.push_back(node.pos+glm::vec3(-CH, -CV, CL));

        for (auto& v : verts) {
            glm::vec3 delta = v - model.center.pos;
            float     r     = model.distH;                    // sphere “radius”
            float     d     = glm::length(delta);

            if (d < r) {
                glm::vec3 n = (d > 1e-6f) ? delta / d : glm::vec3(0.0f, 1.0f, 0.0f);
                float pen   = r - d;

                node.pos += n * pen;
                float vn = glm::dot(node.vel, n);
                if (vn < 0.0f) {
                    glm::vec3 tangentVel = node.vel - vn * n;  // Remove normal component
                    node.vel -= friction * tangentVel;  // Apply friction

                    // Bounce (normal component)
                    node.vel -= (1.0f + model.bounciness) * vn * n;
                }
            }
        }
        return;
    }
}

void handleCollisionSM(Node& node, float radius, Model& model) {
    glm::vec3 delta = node.pos - model.center.pos;
    glm::vec3 dir = glm::normalize(delta);

    glm::vec3 v = node.pos + dir * radius;
    // ===============  A. axis-aligned cube  ==================================
    if (model.type == 1) {

        float cx = model.center.pos.x;
        float cy = model.center.pos.y;
        float cz = model.center.pos.z;
        float H  = model.distH;           // half-width  (X)
        float V  = model.distV;           // half-height (Y)
        float L  = model.distL;           // half-length (Z)

        // -------- Sphere vs Box Collision -------------------------------------
        float sx = node.pos.x;
        float sy = node.pos.y;
        float sz = node.pos.z;

        float px = std::max(cx - H, std::min(sx, cx + H));
        float py = std::max(cy - V, std::min(sy, cy + V));
        float pz = std::max(cz - L, std::min(sz, cz + L));

        glm::vec3 closestPoint(px, py, pz);
        glm::vec3 delta = node.pos - closestPoint;
        float     d     = glm::length(delta);

        if (d < radius) {
            glm::vec3 n = (d > 1e-6f) ? delta / d : glm::vec3(0.0f, 1.0f, 0.0f);
            float pen   = radius - d;

            node.pos += n * pen;
            float vn = glm::dot(node.vel, n);
            if (vn < 0.0f) {
                glm::vec3 tangentVel = node.vel - vn * n;  // Remove normal component
                node.vel -= friction * tangentVel;  // Apply friction

                // Bounce (normal component)
                node.vel -= (1.0f + model.bounciness) * vn * n;
                //node.vel -= (1.0f + model.bounciness) * vn * n;
                //node.vel *= glm::vec3 (1.0f) - friction;
                //node.vel = model.bounciness * glm::cross(node.vel, n) ;
            }
        }

        return;
    }

    if (model.type == 2) {
        glm::vec3 delta = v - model.center.pos;
        float     r     = model.distH;                    // sphere “radius”
        float     d     = glm::length(delta);

        if (d < r)                                       // penetration?
        {
            glm::vec3 n = (d > 1e-6f) ? delta / d : glm::vec3(0.0f, 1.0f, 0.0f);
            float pen   = r - d + radius;

            node.pos += n * pen;
            float vn = glm::dot(node.vel, n);
            if (vn < 0.0f) {
                //node.vel -= (1.0f + model.bounciness) * vn * n;
                glm::vec3 tangentVel = node.vel - vn * n;  // Remove normal component
                node.vel -= friction * tangentVel;  // Apply friction

                // Bounce (normal component)
                node.vel -= (1.0f + model.bounciness) * vn * n;
            }
        }
        return;
    }
}

float facesAngle(glm::vec3 Nl, glm::vec3 Nr, glm::vec3 Em, float *ArcCosSign = 0) {
    float result;
    float cosAngle = glm::dot(Nl, Nr);
    if (glm::dot(cross(Nl,Nr), Em) < 0.0f) {
        result = 2.0*M_PI - acos(cosAngle);
        *ArcCosSign = -1.0f;
    }
    else {
        result = acos(cosAngle);
        *ArcCosSign = 1.0f;
    }
    return result;
}


void updateSoftNodes(float dt, Model& model) {
    const float dt_sub = dt / model.subStep;
    const vector<unsigned int>& indices = model.faces;


    //model.center.vel += dt * gravity;
    model.center.predicted_pos = model.center.pos;

    model.lambda = 0.0f;

    for (Spring& s : model.springs) {
        s.lambda = 0.0f;
    }

    for (Tetra& t : model.tetras) {
        t.lambda = 0.0f;
    }

    for (AngledSpring& s : model.angledSprings) {
        s.lambda = 0.0f;
    }

    for (int step = 0; step < model.subStep; ++step) {
        //Apply gravity and predict positions
        for (Node& n : model.nodes) {
            n.vel += dt_sub * gravity; // Apply small gravity step
            n.predicted_pos = n.pos + dt_sub * n.vel;
            //printf("Point position: %f %f %f \n", n.predicted_pos.x, n.predicted_pos.y, n.predicted_pos.z);
        }

        // === SPRING CONSTRAINTS ===
        for (Spring& s : model.springs) {
            Node& A = model.nodes[s.A];
            Node& B = model.nodes[s.B];
            glm::vec3 delta = A.predicted_pos - B.predicted_pos;
            float len = glm::length(delta);
            if (len < 1e-6f) continue;

            float C = len - s.restLength;
            float w1 = 1.0f / A.mass;
            float w2 = 1.0f / B.mass;
            float wSum = w1 + w2;

            glm::vec3 deltaC1 = glm::normalize(delta);
            glm::vec3 deltaC2 = -deltaC1;

            float weirdAlpha = s.compliance / (dt_sub * dt_sub);

            float deltaLambda = ((-C - weirdAlpha * s.lambda) / (wSum + weirdAlpha));

            A.predicted_pos += w1 * deltaC1 * deltaLambda;
            B.predicted_pos += w2 * deltaC2 * deltaLambda;

            s.lambda += deltaLambda;
        }


        if (!model.tetra) {
            // === VOLUME CONSTRAINT ===
            float currentVolume = calculatePredictedVolume(model);
            float C = currentVolume - model.volume;

            //printf("Current Volume: %f Rest Volume: %f\n", currentVolume, model.volume);

            if (fabs(C) > 1e-6f) {
                vector<glm::vec3> jJ(model.nodes.size(), glm::vec3(0.0f));

                for (int i = 0; i < indices.size(); i += 3) {
                    int I0 = indices[i];
                    int I1 = indices[i + 1];
                    int I2 = indices[i + 2];

                    glm::vec3 p0 = model.nodes[I0].predicted_pos;
                    glm::vec3 p1 = model.nodes[I1].predicted_pos;
                    glm::vec3 p2 = model.nodes[I2].predicted_pos;

                    jJ[I0] += glm::cross(p1, p2) / 6.0f;
                    jJ[I1] += glm::cross(p2, p0) / 6.0f;
                    jJ[I2] += glm::cross(p0, p1) / 6.0f;
                }
                float weirdAlpha = model.compliance / (dt_sub * dt_sub);
                float denum = weirdAlpha;

                for (int i = 0; i < model.nodes.size(); i++) {
                    Node& n = model.nodes[i];
                    denum += (1.0f/n.mass) * glm::dot(jJ[i], jJ[i]);
                }

                if (fabs(denum) > 1e-6f) {
                    float deltaLambda = (-C - weirdAlpha*(model.lambda)) / denum;

                    for (int i = 0; i < model.nodes.size(); i++) {
                        Node& n = model.nodes[i];
                        n.predicted_pos += ((1.0f/n.mass)*deltaLambda) * jJ[i];
                    }

                    model.lambda += deltaLambda;
                }
            }

            // === BENDING ===
            for (AngledSpring& s : model.angledSprings) {
                Node& N0 = model.nodes[s.edge.x];
                Node& N1 = model.nodes[s.edge.y];
                Node& N2 = model.nodes[s.A];
                Node& N3 = model.nodes[s.B];

                glm::vec3 P0 = N0.predicted_pos;
                glm::vec3 P1 = N1.predicted_pos;
                glm::vec3 P2 = N2.predicted_pos;
                glm::vec3 P3 = N3.predicted_pos;

                glm::vec3 Em = P1 - P0;
                glm::vec3 El = P2 - P0;
                glm::vec3 Er = P3 - P0;

                glm::vec3 Nl = glm::cross(El, Em);
                glm::vec3 Nr = glm::cross(Er, Em);

                float Ll = glm::length(Nl);
                float Lr = glm::length(Nr);

                if (fabs(Ll) > 1e-6f && fabs(Lr) > 1e-6f) {
                    float cosAngle = calculateAngle(Nl, Nr);
                    float ArcCosSign = 1.0f;
                    float currentAngle = facesAngle(Nl, Nr, Em, &ArcCosSign);
                    float deltaAngle = currentAngle - s.restAngle;

                    if (fabs(deltaAngle) > 1e-6f) {
                        glm::vec3 JP1 = ((glm::cross(Er, Nl) + cosAngle*glm::cross(Nr, Er)))/Lr + (glm::cross(El, Nr) + cosAngle*glm::cross(Nl, El));
                        glm::vec3 JP2 = (glm::cross(Nr, Em) - cosAngle*glm::cross(Nl, Em))/Ll;
                        glm::vec3 JP3 = (glm::cross(Nl, Em) - cosAngle*glm::cross(Nr, Em))/Lr;
                        glm::vec3 JP0 = -JP1 - JP2 - JP3;

                        float RadicanDenomF = 1.0f - cosAngle*cosAngle;
                        float DenomF = -ArcCosSign*sqrt(RadicanDenomF);


                        float W0 = 1.0f / N0.mass;
                        float W1 = 1.0f / N1.mass;
                        float W2 = 1.0f / N2.mass;
                        float W3 = 1.0f / N3.mass;

                        float weirdAlpha = s.compliance / (dt_sub * dt_sub);

                        float Denom = W0 * glm::dot(JP0, JP0) + W1 * glm::dot(JP1, JP1) + W2 * glm::dot(JP2, JP2) + W3 * glm::dot(JP3, JP3) + weirdAlpha*RadicanDenomF;

                        if (fabs(Denom) > 1e-6f) {
                            float deltaLambda = (-deltaAngle - weirdAlpha*(s.lambda))/Denom;
                            N0.predicted_pos += (W0*(DenomF*DenomF)*deltaLambda)*JP0;
                            N1.predicted_pos += (W1*(DenomF*DenomF)*deltaLambda)*JP1;
                            N2.predicted_pos += (W2*(DenomF*DenomF)*deltaLambda)*JP2;
                            N3.predicted_pos += (W3*(DenomF*DenomF)*deltaLambda)*JP3;

                            s.lambda += deltaLambda*RadicanDenomF;
                        }
                    }
                }
            }
        }
        else {
            // === VOLUME CONSTRAINT ===
            for (Tetra& t : model.tetras) {
                float currentVolume = calculatePredictedTetraVolume(t, model);
                float C = currentVolume - t.volume;

                if (fabs(C) > 1e-6f) {
                    std::vector<glm::vec3> jJ(4, glm::vec3(0.0f)); // gradient of constraint w.r.t each node

                    // Get indices of the tetrahedron's vertices
                    int I0 = t.out[0];
                    int I1 = t.out[1];
                    int I2 = t.out[2];
                    int I3 = t.in;

                    glm::vec3 p0 = model.nodes[I0].predicted_pos;
                    glm::vec3 p1 = model.nodes[I1].predicted_pos;
                    glm::vec3 p2 = model.nodes[I2].predicted_pos;
                    glm::vec3 p3 = model.nodes[I3].predicted_pos;

                    jJ[0] = glm::cross(p1 - p2, p3 - p2) / 6.0f;
                    jJ[1] = glm::cross(p2 - p0, p3 - p0) / 6.0f;
                    jJ[2] = glm::cross(p0 - p1, p3 - p1) / 6.0f;
                    jJ[3] = glm::cross(p1 - p0, p2 - p0) / 6.0f;

                    float weirdAlpha = model.compliance / (dt_sub * dt_sub);
                    float denum = weirdAlpha;

                    // Compute denominator
                    denum += (1.0f / model.nodes[I0].mass) * glm::dot(jJ[0], jJ[0]);
                    denum += (1.0f / model.nodes[I1].mass) * glm::dot(jJ[1], jJ[1]);
                    denum += (1.0f / model.nodes[I2].mass) * glm::dot(jJ[2], jJ[2]);
                    denum += (1.0f / model.nodes[I3].mass) * glm::dot(jJ[3], jJ[3]);

                    if (fabs(denum) > 1e-6f) {
                        float deltaLambda = (-C - weirdAlpha * model.lambda) / denum;

                        model.nodes[I0].predicted_pos += (1.0f / model.nodes[I0].mass) * deltaLambda * jJ[0];
                        model.nodes[I1].predicted_pos += (1.0f / model.nodes[I1].mass) * deltaLambda * jJ[1];
                        model.nodes[I2].predicted_pos += (1.0f / model.nodes[I2].mass) * deltaLambda * jJ[2];
                        model.nodes[I3].predicted_pos += (1.0f / model.nodes[I3].mass) * deltaLambda * jJ[3];

                        t.lambda += deltaLambda;
                    }
                }
            }
        }



        // === COLLISION ===
        handleCollision(model);
        for (size_t i = 0; i < model.nodes.size(); ++i) {
            Node& n = model.nodes[i];
            n.vel = (n.predicted_pos - n.pos) / dt_sub;
            n.pos = n.predicted_pos;
            n.vel *= 0.9999f;
            n.vel = n.vel * (n.normal * friction + (glm::vec3 (1.0f) - n.normal));
            if (n.hasCollided) {
                if (model.bounciness != 0.0f) {
                    n.vel = glm::cross(n.normal, n.vel) * model.bounciness;
                }
                else {
                    n.vel *= 0.0f;
                }
                n.hasCollided = false;
            }
        }

    }
    
    model.center.vel = (model.center.predicted_pos - model.center.pos) / dt;
    model.center.pos = model.center.predicted_pos;

    computeVertexNormals(model);
    updateGPUpositionsAndNormals(model);

    createBB(model);
    //updateBVH(model);

    //if (frame % 10 == 0) {
        //updateBVH(model);  // Full update every 10 frames
    //}
    //else if (frame % 5 == 0) {
        //updateBVHLeafs(model);  // Partial update every 5 frames (but not on frames divisible by 10)
    //}
}




void updateRigidNodes(float dt, Model& model) {
    Node& center = model.center;

    center.vel += dt * gravity;
    center.predicted_pos = center.pos + dt * center.vel; // Predict new position


    for (Model& m : models) {
        if (&m != &model) {
            if (model.type == 1) {
                handleCollisionBM(center, model.distH, model.distV, model.distL, m);
            }
            else {
                if (model.type == 2) {
                    handleCollisionSM(center, model.distH, m);
                }
            }
        }
    }

    if (center.predicted_pos != center.pos) {
        glm::vec3 deltaPos = center.predicted_pos - center.pos;
        for (Node& n : model.nodes) {
            n.pos += deltaPos;
        }
        center.vel = (center.predicted_pos - center.pos) / dt;
        center.pos = center.predicted_pos;

        updateGPUpositionsAndNormals(model);
    }
    createBB(model);
}

void updateNodes(float dt, Model& model) {
    if (model.type == 0) {
        updateSoftNodes(dt, model);
    }
    else {
        updateRigidNodes(dt, model);
    }
}


void drawModel(const Model &model)
{
    const GLModel &g = model.glModel;
    if (!g.vao) return;

    if (model.wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }


    glBindVertexArray(g.vao);
    glDrawElements(GL_TRIANGLES, g.indexCount, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}



void changeSize(int w, int h) {
    if (h == 0) h = 1;
    float ratio = (float)w / h;
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);
    gluPerspective(45.0f, ratio, 1.0f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
}

void renderScene(void) {
    float fps;
    int time;
    char s[64];

    currentTime = glutGet(GLUT_ELAPSED_TIME);

    float dt = currentTime - previousTime;
    previousTime = currentTime;

    dt /= 1000.0f;

    const float MAX_DT = 1.0f / 30.0f; // Max of 33ms (like 30fps)
    dt = std::min(dt, MAX_DT);
    //dt = 1.0f / 480.0f;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //glLoadIdentity();
    //gluLookAt(CameraX + k * DX, CameraY + k * DY, CameraZ + k * DZ, 0, 0, 0, 0.0f, 1.0f, 0.0f);
    glUseProgram(shaderProgram);



    if (!paused && dt > 0.00001f) {
        for (Model& m : models) {
            if (m.update) {
                updateNodes(dt, m);
            }
        }
    }

    glm::mat4 model = glm::mat4(1.0f);
    //glm::mat4 view = glm::lookAt(glm::vec3(0,0,0), glm::vec3(0, 1, 0), glm::vec3(0, 1, 0));
    glm::mat4 view = glm::lookAt(glm::vec3(CameraX + k * DX, CameraY + k * DY, CameraZ + k * DZ), glm::vec3(0), glm::vec3(0, 1, 0));
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1.0f, 0.1f, 100.0f);

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    glUniform3f(glGetUniformLocation(shaderProgram, "lightPos"), 0.0f, 10.0f, 0.0f);
    glUniform3f(glGetUniformLocation(shaderProgram, "viewPos"), CameraX + k * DX, CameraY + k * DY, CameraZ + k * DZ);
    glUniform3f(glGetUniformLocation(shaderProgram, "lightColor"), 1.0f, 1.0f, 1.0f);

    for (Model& m : models) {
        glUniform3f(glGetUniformLocation(shaderProgram, "objectColor"), m.color.x, m.color.y, m.color.z);

        drawModel(m);
    }

    frame++;
    time=glutGet(GLUT_ELAPSED_TIME);
    if (time - timebase > 1000) {
        fps = frame*1000.0/(time-timebase);
        timebase = time;
        frame = 0;
        sprintf(s, "FPS: %f6.2", fps);
        glutSetWindowTitle(s);
    }

    //printf("Sphere top node pos %f %f %f \n", models[0].nodes[0].pos.x, models[0].nodes[0].pos.y, models[0].nodes[0].pos.z);
    glutSwapBuffers();
}

void keyManage(unsigned char key, int x, int y) {
    switch (key) {
        case 'd': case 'D':
            alpha += M_PI / speed;
            CameraZ = k * cos(::beta) * cos(alpha);
            CameraX = k * cos(::beta) * sin(alpha);
            CameraY = k * sin(::beta);

            break;

        case 'a': case 'A':
            alpha -= M_PI / speed;
            CameraZ = k * cos(::beta) * cos(alpha);
            CameraX = k * cos(::beta) * sin(alpha);
            CameraY = k * sin(::beta);
            break;

        case 'w': case 'W':
            if (::beta <= M_PI / 2) {
                ::beta += M_PI / speed;
            }
            CameraZ = k * cos(::beta) * cos(alpha);
            CameraX = k * cos(::beta) * sin(alpha);
            CameraY = k * sin(::beta);
            break;

        case 's': case 'S':
            if (::beta >= -M_PI / 2) {
                ::beta -= M_PI / speed;
            }
            CameraZ = k * cos(::beta) * cos(alpha);
            CameraX = k * cos(::beta) * sin(alpha);
            CameraY = k * sin(::beta);
            break;

        case 'i': case 'I':
            k += 1.0f;
            CameraZ = k * cos(::beta) * cos(alpha);
            CameraX = k * cos(::beta) * sin(alpha);
            CameraY = k * sin(::beta);
            break;
        case 'o': case 'O':
            k -= 1.0f;
            CameraZ = k * cos(::beta) * cos(alpha);
            CameraX = k * cos(::beta) * sin(alpha);
            CameraY = k * sin(::beta);
            break;
        case 'e': case 'E':
            if (speed >0) {
                speed -= 25;
            }
            if (speed <= 0) {
                speed = 1;
            }
            break;
        case 'q': case 'Q':
            speed += 25;
            break;
        case 'p': case 'P':
            paused = !paused;
            if (paused) {
                pauseB = glutGet(GLUT_ELAPSED_TIME);
            }
            else {
                previousTime = glutGet(GLUT_ELAPSED_TIME);
                currentTime = glutGet(GLUT_ELAPSED_TIME);
                //pauseA = glutGet(GLUT_ELAPSED_TIME);
                //float dif = pauseA - pauseB;
                //currentTime += dif;
                //previousTime += currentTime;
            }
            break;
    }

    glutPostRedisplay(); // Request scene update
}

GLuint compileShader(const char* path, GLenum type) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open shader file: " << path << std::endl;
        return 0;
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string src = buffer.str();
    const char* csrc = src.c_str();

    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &csrc, NULL);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "Shader compilation failed for " << path << ":\n" << infoLog << std::endl;
        return 0;
    }
    return shader;
}


GLuint createShaderProgram(const char* vertPath, const char* fragPath) {
    GLuint vert = compileShader(vertPath, GL_VERTEX_SHADER);
    if (vert == 0) return 0;

    GLuint frag = compileShader(fragPath, GL_FRAGMENT_SHADER);
    if (frag == 0) {
        glDeleteShader(vert);
        return 0;
    }

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vert);
    glAttachShader(prog, frag);
    glLinkProgram(prog);

    GLint success;
    glGetProgramiv(prog, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(prog, 512, NULL, infoLog);
        std::cerr << "Shader program linking failed:\n" << infoLog << std::endl;
        glDeleteShader(vert);
        glDeleteShader(frag);
        glDeleteProgram(prog);
        return 0;
    }

    glDeleteShader(vert);
    glDeleteShader(frag);

    return prog;
}

int main(int argc, char **argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(800, 800);
    glutCreateWindow("CG@DI-UM");

    // Ensure the camera looks at the origin at start
    CameraX = k * cos(::beta) * sin(alpha);
    CameraY = k * sin(::beta);
    CameraZ = k * cos(::beta) * cos(alpha);


    glewInit();

    shaderProgram = createShaderProgram("vertex.glsl", "fragment.glsl");

    previousTime = glutGet(GLUT_ELAPSED_TIME);
    currentTime = glutGet(GLUT_ELAPSED_TIME);

    //models.push_back(makeRigidSphere(1, 20, 20,1, 50, 0,1, 0, glm::vec3(1.0f,1.0f,0.0f),true));
    //models.push_back(makeSoftSphere(1, 20, 20, 2.0f, 1, 0.5f, 0.0f, 0.0f, 0.0f,0.0f, 0,1,0,glm::vec3(1.0f,0.0f,0.0f), true, true, true));
    //models.push_back(makeSoftSphere(1, 20, 20, 2.0f, 1, 0.5f, 0.0f, 0.0f, 0.0f,0.0f, 0,10,0,glm::vec3(1.0f,1.0f,0.0f), true, true, true));
    models.push_back(makeSoftIcoSphere(1, 2, 2.0f, 1, 0.5f, 0.00000f, 0.00f, 0.00f,0.0f, 0,5,0,glm::vec3(1.0f,1.0f,0.0f), false, true, true));
    models.push_back(makeSoftIcoSphere(1, 3, 2.0f, 1, 0.5f, 0.000001f, 0.001f, 0.001f,0.0f, 0,10,0,glm::vec3(1.0f,1.0f,0.5f), false, true, true));
    //models.push_back(makeSoftSphere(1, 20, 20,1, 100, 2, 0, 10, 0, 20));
    models.push_back(makeRigidBox(2, 2, false, 1, 1.0f, 0, -2.2, 0, 200, 2, 100));
    //models.push_back(makeSoftBox(5, 5, 2.0f, 1, 0.5f,0.000001f, 0.01f, 0.01f, 0, 0, 1.0,0,3,1,3,glm::vec3(1.0f,1.0f,0.5f), false, true, true));
    //models.push_back(makeSoftBox(5, 5, 2.0f, 1, 0.5f,0.00000f, 0.0f, 0.0f, 0, 0, 1.0,0,3,1,3,glm::vec3(0.5f,1.0f,0.5f), true, true, true));
    //models.push_back(makeRigidBox(20, 20, false, 1, 0.5,0, 2, 0, 2, 1, 1,glm::vec3(0.0f,1.0f,0.0f), false, false));
    //models.push_back(makeSoftSphere(3, 20, 20,1, 50, 1, 0, 3, 0, 10, true));

    glutDisplayFunc(renderScene);
    glutIdleFunc(renderScene);
    glutReshapeFunc(changeSize);
    glutKeyboardFunc(keyManage);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glutMainLoop();


    return 0;
}

// TIP See CLion help at <a
// href="https://www.jetbrains.com/help/clion/">jetbrains.com/help/clion/</a>.
//  Also, you can try interactive lessons for CLion by selecting
//  'Help | Learn IDE Features' from the main menu.