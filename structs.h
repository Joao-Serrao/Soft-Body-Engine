#ifndef STRUCTS_H
#define STRUCTS_H

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/hash.hpp>
#include <cmath>
#include <vector>
#include <random>
#include <unordered_map>
#include <memory>
#include <GL/glew.h>
#include <GL/glut.h>
#include <json.hpp>

using json = nlohmann::json;

using namespace std;

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

// Struct used to represent a BB
struct BB {
    float minX = std::numeric_limits<float>::infinity(), maxX = -std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity(), maxY = -std::numeric_limits<float>::infinity();
    float minZ = std::numeric_limits<float>::infinity(), maxZ = -std::numeric_limits<float>::infinity();
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



// Struct to represent the Constraint between Nodes
struct Spring {
    int A;  // Index of First Node
    int B;  // Index of Second Node
    float compliance = 0.0f;  // Inverse Stifness (smaller = more stiff)
    float restLength = 1.0f;  // Distance between Nodes to try to achieve
    float lambda = 0.0f;  // Lambda saved between SubSteps
    bool in = true;
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

// Struct used to represent a Tetrahedrons
struct Tetra {
    int in;  // Index of the point inside the sphere
    vector<unsigned int> out;  // Indexs of the Outer Points
    float volume;  // Volume between Nodes to try to achieve
    float lambda = 0.0f;  // Lambda saved between SubSteps
    bool surface = false;
    BB bb;
};

// Struct used to represent any Model
struct Model {
    //-----Nodes Info-----//
    vector<float> nodesMass;
    vector<glm::vec3> nodesPos;  // List of the Nodes (innner and outer)
    vector<glm::vec3> nodesPredictedPos;
    vector<glm::vec3> nodesVel;
    vector<glm::vec3> nodesNor;
    vector<unsigned int> collided;
    vector<glm::vec3> collidedNor;
    //-----Constraints Vectors-----//
    vector<unsigned int> faces;  // List of the Index that make up Faces
    vector<Spring> springs;  // List of the Constraints
    vector<AngledSpring> angledSprings;  // List of the Angle Constraints
    vector<Tetra> tetras;  // List of the Tetrahedrons
    unordered_map<unsigned int,vector<unsigned int>> springsMap;
    //-----Model Info-----//
    int type; // 0 - Soft; 1 - Box; 2 - Sphere
    float distV, distH, distL;  // Distance between Center and every Plane for Box type; Radius for Sphere type;
    float bounciness = 0.0f;  // Bounciness of the surface (right now is also friction)
    glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f);  // Color vector
    bool wireframe = false;  // Render in Wireframe Mode
    bool update = false;  // Update Positions
    bool tetra = false;  // Uses Tetrahedrons
    GLModel glModel;  // Information for VBOs
    //-----Model Constraints Values-----//
    float volumeCompliance = 0.0f;  // Inverse Stifness for Volume (smaller = more stiff)
    float volume = 0.0f;  // Volume between Nodes to try to achieve
    float lambda = 0.0f;  // Lambda saved between SubSteps
    float subStep = 1.0f;  // Number of SubSteps per Frame
    BB bb;  // Defines the Bounding Box
    vector<BVH> bvhs;
};

struct LevelModel {
    Model model;
    bool customize = false;
    glm::vec3 position = glm::vec3(0.0f);
    bool wireframe = false;
    bool update = false;
    float springCompliance = 0.0f;
    float springCenterCompliance = 0.0f;
    float angleCompliance = 0.0f;
    float volumeCompliance = 0.0f;
};

struct Level {
    vector<Model> models;
    Model floor;
    int x;
    int y;
};

namespace glm {
    inline void to_json(json& j, const glm::vec2& v) {
        j = json::array({v.x, v.y});
    }

    inline void from_json(const json& j, glm::vec2& v) {
        v = glm::vec2(j.at(0).get<float>(), j.at(1).get<float>());
    }

    inline void to_json(json& j, const glm::vec3& v) {
        j = json::array({v.x, v.y, v.z});
    }

    inline void from_json(const json& j, glm::vec3& v) {
        v = glm::vec3(j.at(0).get<float>(), j.at(1).get<float>(), j.at(2).get<float>());
    }
}


inline void to_json(json& j, const Spring& s) {
    j = json{{"A", s.A}, {"B", s.B}, {"compliance", s.compliance}, {"restLength", s.restLength}, {"lambda", s.lambda}, {"in", s.in}};
}

inline void from_json(const json& j, Spring& s) {
    j.at("A").get_to(s.A);
    j.at("B").get_to(s.B);
    j.at("compliance").get_to(s.compliance);
    j.at("restLength").get_to(s.restLength);
    j.at("lambda").get_to(s.lambda);
    j.at("in").get_to(s.in);
}



inline void to_json(json& j, const BB& bb) {
    j = json{
            {"minX", bb.minX}, {"maxX", bb.maxX},
            {"minY", bb.minY}, {"maxY", bb.maxY},
            {"minZ", bb.minZ}, {"maxZ", bb.maxZ}
    };
}

inline void from_json(const json& j, BB& bb) {
    j.at("minX").get_to(bb.minX); j.at("maxX").get_to(bb.maxX);
    j.at("minY").get_to(bb.minY); j.at("maxY").get_to(bb.maxY);
    j.at("minZ").get_to(bb.minZ); j.at("maxZ").get_to(bb.maxZ);
}


// AngledSpring
inline void to_json(json &j, const AngledSpring &a) {
    j = json{{"A", a.A}, {"B", a.B}, {"edge", a.edge}, {"compliance", a.compliance}, {"restAngle", a.restAngle}, {"lambda", a.lambda}};
}

inline void from_json(const json &j, AngledSpring & a) {
    j.at("A").get_to(a.A);
    j.at("B").get_to(a.B);
    j.at("edge").get_to(a.edge);
    j.at("compliance").get_to(a.compliance);
    j.at("restAngle").get_to(a.restAngle);
    j.at("lambda").get_to(a.lambda);
}

// Tetra
inline void to_json(json& j, const Tetra& t) {
    j = json{
        {"in", t.in},
        {"out", t.out},
        {"volume", t.volume},
        {"lambda", t.lambda},
        {"surface", t.surface},
        {"bb", t.bb}
    };
}

inline void from_json(const json& j, Tetra& t) {
    j.at("in").get_to(t.in);
    j.at("out").get_to(t.out);
    j.at("volume").get_to(t.volume);
    j.at("lambda").get_to(t.lambda);
    j.at("surface").get_to(t.surface);
    j.at("bb").get_to(t.bb);
}

// BVH
inline void to_json(json& j, const BVH& b) {
    j = json{
        {"bb", b.bb},
        {"self", b.self},
        {"left", b.left},
        {"right", b.right},
        {"tetra", b.tetra}
    };
}

inline void from_json(const json& j, BVH& b) {
    j.at("bb").get_to(b.bb);
    j.at("self").get_to(b.self);
    j.at("left").get_to(b.left);
    j.at("right").get_to(b.right);
    j.at("tetra").get_to(b.tetra);
}

// Model
inline void to_json(json& j, const Model& m) {
    j = json{
        {"nodesMass", m.nodesMass},
        {"nodesPos", m.nodesPos},
        {"nodesPredictedPos", m.nodesPredictedPos},
        {"nodesVel", m.nodesVel},
        {"nodesNor", m.nodesNor},
        {"collided", m.collided},
        {"collidedNor", m.collidedNor},
        {"faces", m.faces},
        {"springs", m.springs},
        {"angledSprings", m.angledSprings},
        {"tetras", m.tetras},
        {"springsMap", m.springsMap},
        {"type", m.type},
        {"distV", m.distV},
        {"distH", m.distH},
        {"distL", m.distL},
        {"bounciness", m.bounciness},
        {"color", m.color},
        {"wireframe", m.wireframe},
        {"update", m.update},
        {"tetra", m.tetra},
        {"volumeCompliance", m.volumeCompliance},
        {"volume", m.volume},
        {"lambda", m.lambda},
        {"subStep", m.subStep},
        {"bb", m.bb},
        {"bvhs", m.bvhs}
    };
}

inline void from_json(const json& j, Model& m) {
    j.at("nodesMass").get_to(m.nodesMass);
    j.at("nodesPos").get_to(m.nodesPos);
    j.at("nodesPredictedPos").get_to(m.nodesPredictedPos);
    j.at("nodesVel").get_to(m.nodesVel);
    j.at("nodesNor").get_to(m.nodesNor);
    j.at("collided").get_to(m.collided);
    j.at("collidedNor").get_to(m.collidedNor);
    j.at("faces").get_to(m.faces);
    j.at("springs").get_to(m.springs);
    j.at("angledSprings").get_to(m.angledSprings);
    j.at("tetras").get_to(m.tetras);
    j.at("springsMap").get_to(m.springsMap);
    j.at("type").get_to(m.type);
    j.at("distV").get_to(m.distV);
    j.at("distH").get_to(m.distH);
    j.at("distL").get_to(m.distL);
    j.at("bounciness").get_to(m.bounciness);
    j.at("color").get_to(m.color);
    j.at("wireframe").get_to(m.wireframe);
    j.at("update").get_to(m.update);
    j.at("tetra").get_to(m.tetra);
    j.at("volumeCompliance").get_to(m.volumeCompliance);
    j.at("volume").get_to(m.volume);
    j.at("lambda").get_to(m.lambda);
    j.at("subStep").get_to(m.subStep);
    j.at("bb").get_to(m.bb);
    j.at("bvhs").get_to(m.bvhs);
}


#endif


