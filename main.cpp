#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>


#include <GL/glew.h>
#include <GL/glut.h>


#define USE_MATH_DEFINES

#include <corecrt_math_defines.h>
#include <filesystem>
#include <omp.h>

#include "supportingLibrary.h"
#include "packages/json-develop/single_include/nlohmann/json.hpp"


using namespace std;
namespace fs = filesystem;

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

// Statistics
float springTime = 0.0f;
int springFrame = 0;

float normalTime = 0.0f;
int normalFrame = 0;

float volumeTime = 0.0f;
int volumeFrame = 0;

float collisionTime = 0.0f;
int collisionFrame = 0;

// Delta Time
float currentTime = 0.0f;
float previousTime = 0.0f;

bool paused = false;

// Constants
constexpr float EPSILON = 1e-6f;  // Float Precision
constexpr float GOLDEN_RATIO = 1.618f;
constexpr glm::vec3 ORIGIN = glm::vec3 (0.0f);

// Forces
constexpr glm::vec3 gravity = glm::vec3(0.0f, -9.81f, 0.0f);  // Gravity vector
constexpr glm::vec3 friction = glm::vec3(0.99f, 0.99f, 0.99f);  // General vector for friction (test)

// Struct for Vertex used in VBOs
struct Vertex {
    glm::vec3 pos;    // Position
    glm::vec3 normal; // Normal (used for light)
};

Level loadedLevel;  // Level that is currently loaded
vector<Model> models;  // List of Global Models
vector<int> updateModels;  // List of the index of Models that are updated
GLuint shaderProgram;



void uploadModelToGPU(Model &model) {
    if (model.nodesPos.empty() || model.faces.empty()) return;

    vector<Vertex> verts(model.nodesPos.size());

    for (size_t i = 0; i < model.nodesPos.size(); ++i) {
        verts[i].pos = model.nodesPos[i];
        verts[i].normal = model.nodesNor[i];
    }

    auto &[vao, vboVertices, vboIndices, indexCount] = model.glModel;

    // Generate VAO
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // VBO for vertex data
    glGenBuffers(1, &vboVertices);
    glBindBuffer(GL_ARRAY_BUFFER, vboVertices);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex) * verts.size(), verts.data(), GL_DYNAMIC_DRAW);

    // VBO for indices
    glGenBuffers(1, &vboIndices);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboIndices);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, model.faces.size() * sizeof(unsigned int), model.faces.data(), GL_STATIC_DRAW);

    // Vertex attributes
    glEnableVertexAttribArray(0); // position
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), static_cast<void *>(nullptr));

    glEnableVertexAttribArray(1); // normal
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), reinterpret_cast<void *>(offsetof(Vertex, normal)));

    // Unbind VAO (optional for safety)
    glBindVertexArray(0);

    indexCount = static_cast<GLsizei>(model.faces.size());
}


//--------------------------------------------------------------
//  Update only positions inside an interleaved VBO – NEW
//--------------------------------------------------------------
void updateGPUpositionsAndNormals(const Model &model) {
    glBindBuffer(GL_ARRAY_BUFFER, model.glModel.vboVertices);

    for (size_t i = 0; i < model.nodesPos.size(); ++i) {
        constexpr GLsizei stride = sizeof(Vertex);
        const auto baseOffset = static_cast<GLsizeiptr>(i * stride);
        glBufferSubData(GL_ARRAY_BUFFER, baseOffset, sizeof(glm::vec3), &model.nodesPos[i]);
        glBufferSubData(GL_ARRAY_BUFFER, baseOffset + sizeof(glm::vec3), sizeof(glm::vec3), &model.nodesNor[i]);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void initializePos(Model &model, const glm::vec3 &pos) {
    for (auto & nodesPos : model.nodesPos) {
        nodesPos += pos;
    }
    model.nodesPredictedPos = model.nodesPos;
    model.nodesVel = vector<glm::vec3>(model.nodesPos.size(), glm::vec3(0.0f));
}

void modifySprings(Model &model, const float compliance, const float centerCompliance) {
    for (Spring& spring : model.springs) {
        if (spring.in) {
            spring.compliance = centerCompliance;
        }
        else {
            spring.compliance = compliance;
        }
    }
}

void modifyAngledSprings(Model &model, const float compliance) {
    for (AngledSpring& angledSpring : model.angledSprings) {
        angledSpring.compliance = compliance;
    }
}

void loadModel(Level &level, const string &path, const bool floor, const glm::vec3 &pos = glm::vec3(0.0f), const bool customize = false, const bool wireframe = false, const bool update = false,
                const float springCompliance = 0.0f, const float springCenterCompliance = 0.0f, const float angleCompliance = 0.0f, const float volumeCompliance = 0.0f) {
    if (fs::exists(path)) {

        ifstream modelFile (path);

        if (modelFile.is_open())
        {
            try {
                json j;
                modelFile >> j;
                Model loaded = j.get<Model>();

                initializePos(loaded, pos);

                constexpr int soft = 0;
                if (customize && loaded.type == soft) {
                    modifySprings(loaded, springCompliance, springCenterCompliance);

                    if (!loaded.tetra) {
                        modifyAngledSprings(loaded, angleCompliance);
                    }

                    loaded.volumeCompliance = volumeCompliance;
                }

                loaded.wireframe = wireframe;
                loaded.update = update;

                createBB(loaded);

                uploadModelToGPU(loaded);
                if (floor) {
                    level.floor = loaded;
                }
                else {
                    level.models.push_back(loaded);
                }
            } catch (const nlohmann::json::exception& e) {
                std::cerr << "Failed to parse model JSON: " << e.what() << '\n';
            }
        }
        else {
            cerr << "Unable to open file";
        }
    }
    else {
        cerr << "Model file doesnt exist: " << path << '\n';
    }
}

void loadLevel(const string& path) {
    if (fs::exists(path)) {

        ifstream levelFile (path);

        if (levelFile.is_open())
        {
            Level level;
            int x, y;
            levelFile >> x >> y;
            level.x = x;
            level.y = y;

            string modelsDir;

            levelFile >> modelsDir;

            if (!fs::exists(modelsDir)) {
                cerr << "Model Directory doesnt exist: " << modelsDir << '\n';
                return;
            }

            int modelNum = 0;
            levelFile >> modelNum;

            bool existFloor = false;

            for (int i = 0; i < modelNum; ++i) {
                string modelName;
                float mX, mY, mZ;
                bool customize = false;

                levelFile >> modelName >> mX >> mY >> mZ >> customize;

                const string modelPath = modelsDir + '/' + modelName + ".json";
                const glm::vec3 pos = glm::vec3(mX, mY, mZ);

                if (customize) {
                    bool wireframe, update;
                    float springCompliance, springCenterCompliance, angleCompliance, volumeCompliance;

                    levelFile >> wireframe >> update >> springCompliance >> springCenterCompliance >> angleCompliance >> volumeCompliance;

                    loadModel(level, modelPath, existFloor, pos, customize, wireframe, update, springCompliance, springCenterCompliance, angleCompliance, volumeCompliance);
                }
                else {
                    loadModel(level, modelPath, existFloor, pos);
                }
            }

            loadedLevel = level;
            models = level.models;

            int i = 0;
            for (Model &model : level.models) {
                if (model.update) {
                    updateModels.push_back(i);
                }
                i++;
            }

            levelFile >> existFloor;

            if (existFloor) {
                string modelName;
                float mX, mY, mZ;
                bool customize = false;

                levelFile >> modelName >> mX >> mY >> mZ >> customize;

                string modelPath = modelsDir + '/' + modelName + ".json";
                glm::vec3 pos = glm::vec3(mX, mY, mZ);

                if (customize) {
                    bool wireframe, update;
                    float springCompliance, springCenterCompliance, angleCompliance, volumeCompliance;

                    levelFile >> wireframe >> update >> springCompliance >> springCenterCompliance >> angleCompliance >> volumeCompliance;

                    loadModel(level, modelPath, existFloor, pos, customize, wireframe, update, springCompliance, springCenterCompliance, angleCompliance, volumeCompliance);
                }
                else {
                    loadModel(level, modelPath, existFloor, pos);
                }
                models.push_back(level.floor);
                if (level.floor.update) {
                    updateModels.push_back(models.size() - 1);
                }
            }
        }
        else {
            cerr << "Unable to open file";
        }
    }
    else {
        cerr << "Level file doesnt exist: " << path << '\n';
    }
}



bool isPointInTriangle(const glm::vec3& pt, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
    const glm::vec3 n = glm::normalize(glm::cross(b - a, c - a));

    // Barycentric technique using cross products
    const glm::vec3 edge0 = b - a;
    const glm::vec3 vp0 = pt - a;
    const glm::vec3 c0 = glm::cross(edge0, vp0);
    if (glm::dot(n, c0) < 0) return false;

    const glm::vec3 edge1 = c - b;
    const glm::vec3 vp1 = pt - b;
    const glm::vec3 c1 = glm::cross(edge1, vp1);
    if (glm::dot(n, c1) < 0) return false;

    const glm::vec3 edge2 = a - c;
    const glm::vec3 vp2 = pt - c;
    const glm::vec3 c2 = glm::cross(edge2, vp2);
    if (glm::dot(n, c2) < 0) return false;

    return true;
}

bool rayIntersectsTriangle(
    const glm::vec3& orig, const glm::vec3& dir,
    const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2,
    float* outT = nullptr) {

    const glm::vec3 edge1 = v1 - v0;
    const glm::vec3 edge2 = v2 - v0;

    const glm::vec3 h = glm::cross(dir, edge2);
    const float a = glm::dot(edge1, h);

    if (fabs(a) < EPSILON)
        return false;  // Backface or parallel — cull


    const float f = 1.0f / a;
    const glm::vec3 s = orig - v0;
    const float u = f * glm::dot(s, h);

    if (u < 0.0f || u > 1.0f)
        return false;

    const glm::vec3 q = glm::cross(s, edge1);
    const float v = f * glm::dot(dir, q);

    if (v < 0.0f || u + v > 1.0f)
        return false;

    const float t = f * glm::dot(edge2, q);
    if (t > EPSILON) {
        if (outT) *outT = t;
        return true;
    }

    return false;
}

bool rayIntersectsBB (const glm::vec3 &p, const glm::vec3 &ray, const BB &bb) {
    float tTemp;

    const float invX = fabs(ray.x) > EPSILON ? 1.0f / ray.x : 1e8f;
    float tminX = (bb.minX - p.x) * invX;
    float tmaxX = (bb.maxX - p.x) * invX;


    if (tminX > tmaxX) {
        tTemp = tmaxX;
        tmaxX = tminX;
        tminX = tTemp;
    }

    const float invY = fabs(ray.y) > EPSILON ? 1.0f / ray.y : 1e8f;
    float tminY = (bb.minY - p.y) * invY;
    float tmaxY = (bb.maxY - p.y) * invY;


    if (tminY > tmaxY) {
        tTemp = tmaxY;
        tmaxY = tminY;
        tminY = tTemp;
    }

    const float invZ = fabs(ray.z) > EPSILON ? 1.0f / ray.z : 1e8f;
    float tminZ = (bb.minZ - p.z) * invZ;
    float tmaxZ = (bb.maxZ - p.z) * invZ;


    if (tminZ > tmaxZ) {
        tTemp = tmaxZ;
        tmaxZ = tminZ;
        tminZ = tTemp;
    }

    const float tMin = max(max(tminX, tminY), tminZ);
    const float tMax = min(min(tmaxX, tmaxY), tmaxZ);

    if (tMin > tMax) {
        return false;
    }
    return true;
}

float distanceToPlaneAlongDirection(const glm::vec3 &rayPoint, const glm::vec3 &rayDir, const glm::vec3 &planePoint, const glm::vec3 &planeNormal){
    const float denom = glm::dot(planeNormal, rayDir);
    if (abs(denom) < 1e-6f) return -1; // Parallel

    const float t = glm::dot(planePoint - rayPoint, planeNormal) / denom;
    return t;
}

glm::vec3 farthestPointInDirection(const Tetra &tetra, const glm::vec3 &dir, const vector<glm::vec3> &nodesPos, const glm::vec3 center) {
    float maxDot = -std::numeric_limits<float>::infinity();
    glm::vec3 bestPoint;

    auto check = [&](const int idx) {
        const glm::vec3 p = nodesPos[idx];
        const float d = glm::dot(p - center, dir);
        if (d > maxDot) {
            maxDot = d;
            bestPoint = p;
        }
    };

    check(tetra.in);
    for (const unsigned int o : tetra.out) check(o);

    return bestPoint;
}


glm::vec3 Support(const Tetra &tetra1, const Tetra &tetra2, const glm::vec3 &dir, const Model &m1, const Model &m2, const glm::vec3 c1, const glm::vec3 c2) {
    const glm::vec3 p1 = farthestPointInDirection(tetra1, dir, m1.nodesPredictedPos, c1);
    const glm::vec3 p2 = farthestPointInDirection(tetra2, dir, m2.nodesPos, c2);
    return p1 - p2;
}

bool lineCase(vector<glm::vec3> &simplex, glm::vec3 &dir) {
    const glm::vec3 A = simplex[0];
    const glm::vec3 B = simplex[1];

    const glm::vec3 AB = B - A;
    const glm::vec3 AO = ORIGIN - A;

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
    const glm::vec3 A = simplex[0];
    const glm::vec3 B = simplex[1];
    const glm::vec3 C = simplex[2];

    const glm::vec3 AB = B - A;
    const glm::vec3 AC = C - A;
    const glm::vec3 AO = ORIGIN - A;

    const glm::vec3 ABC = glm::cross(AB, AC);

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
    return false;
}

bool tetraCase(vector<glm::vec3> &simplex, glm::vec3 &dir) {
    const glm::vec3 A = simplex[0];
    const glm::vec3 B = simplex[1];
    const glm::vec3 C = simplex[2];
    const glm::vec3 D = simplex[3];

    const glm::vec3 AB = B - A;
    const glm::vec3 AC = C - A;
    const glm::vec3 AD = D - A;
    const glm::vec3 AO = ORIGIN - A;

    const glm::vec3 ABC = glm::cross(AB, AC);
    const glm::vec3 ACD = glm::cross(AC, AD);
    const glm::vec3 ADB = glm::cross(AD, AB);

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

bool GJK (const Tetra &tetra1, const Tetra &tetra2, const Model &m1, const Model &m2) {
    const glm::vec3 c1 = (m1.nodesPredictedPos[tetra1.in] + m1.nodesPredictedPos[tetra1.out[0]] + m1.nodesPredictedPos[tetra1.out[1]] + m1.nodesPredictedPos[tetra1.out[2]]) * 0.25f;
    const glm::vec3 c2 = (m2.nodesPos[tetra2.in] + m2.nodesPos[tetra2.out[0]] + m2.nodesPos[tetra2.out[1]] + m2.nodesPos[tetra2.out[2]]) * 0.25f;
    glm::vec3 d = glm::normalize(c1 - c2);
    vector<glm::vec3> simplex;
    simplex.push_back(Support(tetra1, tetra2, d, m1, m2, c1, c2));

    d = ORIGIN - simplex[0];
    int iteration = 0;
    constexpr int maxIterations = 30;
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
    const glm::vec3 a = model.nodesPos[tetra.in];
    const glm::vec3 b = model.nodesPos[tetra.out[0]];
    const glm::vec3 c = model.nodesPos[tetra.out[1]];
    const glm::vec3 d = model.nodesPos[tetra.out[2]];

    const float v0 = glm::dot(glm::cross(b - a, c - a), d - a);
    const float v1 = glm::dot(glm::cross(b - p, c - p), d - p);
    const float v2 = glm::dot(glm::cross(a - p, c - p), d - p);
    const float v3 = glm::dot(glm::cross(a - p, b - p), d - p);
    const float v4 = glm::dot(glm::cross(a - p, b - p), c - p);

    return (v0 > 0 && v1 > 0 && v2 > 0 && v3 > 0 && v4 > 0) ||
           (v0 < 0 && v1 < 0 && v2 < 0 && v3 < 0 && v4 < 0);
}


bool tetraIntersect(const Tetra &a, const Tetra &b, const Model &m1, const Model &m2) {
    for (const unsigned int ai : {static_cast<unsigned int>(a.in), a.out[0], a.out[1], a.out[2]}) {
        if (pointInTetra(m1.nodesPredictedPos[ai], b, m2)) return true;
    }

    for (const unsigned int bi : {static_cast<unsigned int>(b.in), b.out[0], b.out[1], b.out[2]}){
        if (pointInTetra(m2.nodesPos[bi], a, m1)) return true;
    }

    return false;
}


float distanceTetra (const Tetra &tetra1, const Tetra &tetra2, const Model &m1, const Model &m2) {
    float minDist = numeric_limits<float>::infinity();
    for (const unsigned int o1 : tetra1.out) {
        for (const unsigned int o2 : tetra2.out) {
            const float len = glm::length(m1.nodesPredictedPos[o1] - m2.nodesPos[o2]);
            if (len < minDist) {
                minDist = len;
            }
        }
    }
    return minDist;
}

int BVHrayCasting (Tetra &tetra, const BVH &bvh, Model &m1, Model &m2) {
    if (bvh.left == -1 && bvh.right == -1) {
        if (tetraIntersect(tetra, m2.tetras[bvh.tetra], m1, m2)) {
            return bvh.tetra;
        }

        return -1;
    }

    const BVH &left = m2.bvhs[bvh.left];
    const BVH &right = m2.bvhs[bvh.right];

    const bool insideLeft = intersectBB(tetra.bb, left.bb);
    const bool insideRight = intersectBB(tetra.bb, right.bb);

    int result = -1;

    if (insideLeft) {
        result = BVHrayCasting(tetra, left, m1, m2);
    }

    if (insideRight) {
        if (result != -1) {
            const int temp = BVHrayCasting(tetra, right, m1, m2);
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


bool SameSide(const glm::vec3 v1, const glm::vec3 v2, const glm::vec3 v3, const glm::vec3 v4, const glm::vec3 p) {
    const glm::vec3 normal = glm::cross(v2 - v1, v3 - v1);
    const float dotV4 = glm::dot(normal, v4 - v1);
    const float dotP = glm::dot(normal, p - v1);

    return (dotV4 <= 0 && dotP <= 0) || (dotV4 >= 0 && dotP >= 0);
}

bool PointInTetrahedron(const glm::vec3 v1, const glm::vec3 v2, const glm::vec3 v3, const glm::vec3 v4,glm::vec3 p) {
    return SameSide(v1, v2, v3, v4, p) &&
           SameSide(v2, v3, v4, v1, p) &&
           SameSide(v3, v4, v1, v2, p) &&
           SameSide(v4, v1, v2, v3, p);
}

//------------------------------------------------------------------------------
//  HARD-body particle–model collision
//------------------------------------------------------------------------------
void handleCollisionPM(Model &model, const int node, const Model& m) {

    const glm::vec3& pos = model.nodesPredictedPos[node];
    const glm::vec3& previous_pos = model.nodesPos[node];

    if (pos.x > m.bb.minX && pos.x < m.bb.maxX && pos.y > m.bb.minY && pos.y < m.bb.maxY && pos.z > m.bb.minZ && pos.z < m.bb.maxZ) {
        if (m.type == 1) {
            // -------- A. Axis-Aligned Box --------------
            const glm::vec3 c = m.nodesPos[m.nodesPos.size() - 1];
            const float H = m.distH, V = m.distV, L = m.distL;

            if (pos.x > c.x - H && pos.x < c.x + H &&
                pos.y > c.y - V && pos.y < c.y + V &&
                pos.z > c.z - L && pos.z < c.z + L) {


                const float dx = std::min(c.x + H - previous_pos.x, previous_pos.x - (c.x - H));
                const float dy = std::min(c.y + V - previous_pos.y, previous_pos.y - (c.y - V));
                const float dz = std::min(c.z + L - previous_pos.z, previous_pos.z - (c.z - L));

                glm::vec3 n(0.0f);
                float pen = dx;

                n = glm::vec3((pos.x > c.x ? c.x + H : c.x - H), pos.y, pos.z);

                glm::vec3 normal = glm::vec3((pos.x > c.x ? 1.0f : -1.0f), 0.0f, 0.0f);
                if (dy < pen) { pen = dy; n = glm::vec3(pos.x, (pos.y > c.y ? c.y + V : c.y - V), pos.z); normal = glm::vec3(0.0f, (pos.y > c.y ? 1.0f : -1.0f), 0.0f);}
                if (dz < pen) { pen = dz; n = glm::vec3(pos.x, pos.y, (pos.z > c.z ? c.z + L : c.z - L)); normal = glm::vec3(0.0f, 0.0f, (pos.z > c.z ? 1.0f : -1.0f));}

                model.nodesPredictedPos[node] = n;
                model.collided.push_back(node);
                model.collidedNor.push_back(normal);
                }
            return;
        }

        if (m.type == 2) {
            // -------- B. Sphere --------------
            const glm::vec3 delta = pos - m.nodesPos[m.nodesPos.size() - 1];
            const float radius = m.distH;
            const float len = glm::length(delta);

            if (len <= radius) {
                const glm::vec3 normal = glm::normalize(delta);

                model.nodesPredictedPos[node] = m.nodesPos[m.nodesPos.size() - 1] + normal * radius;
                model.collided.push_back(node);
                model.collidedNor.push_back(normal);
            }
        }
    }
}

void handleCollisionSS(Model &m1, Model &m2) {
    #pragma omp parallel
{
    std::vector<unsigned int> local_collided;
    std::vector<glm::vec3> local_collidedNor;

    #pragma omp for schedule(dynamic, 4)
    for (int i = 0; i < m1.tetras.size(); i++) {
        Tetra& t = m1.tetras[i];

        if (intersectBB(t.bb, m2.bb) && t.surface) {
            const int tetra = BVHrayCasting(t, m2.bvhs.back(), m1, m2);

            if (tetra != -1) {
                const glm::vec3 p1 = m2.nodesPos[m2.tetras[tetra].out[0]];

                glm::vec3 n1 = glm::normalize(glm::cross(
                    m2.nodesPos[m2.tetras[tetra].out[1]] - p1,
                    m2.nodesPos[m2.tetras[tetra].out[2]] - p1
                ));

                if (glm::dot(n1, m1.nodesPos[t.in] - m2.nodesPos[m2.tetras[tetra].in]) < 0)
                    n1 = -n1;

                float tMass = (
                    m2.nodesMass[m2.tetras[tetra].out[0]] +
                    m2.nodesMass[m2.tetras[tetra].out[1]] +
                    m2.nodesMass[m2.tetras[tetra].out[2]]
                ) / 3.0f;

                for (const unsigned int o : t.out) {
                    const glm::vec3 p = m1.nodesPredictedPos[o];

                    if (glm::dot(p - p1, n1) < 0) {
                        const float dist = distanceToPlaneAlongDirection(p, n1, p1, n1);
                        const float totalMass = m1.nodesMass[o] + tMass;
                        const float ratio = tMass / totalMass;
                        local_collided.push_back(o);
                        local_collidedNor.push_back(n1);
                        m1.nodesPredictedPos[o] += n1 * ratio * dist;
                    }
                }

                const glm::vec3 p2 = m1.nodesPredictedPos[t.out[0]];
                const glm::vec3 n2 = glm::normalize(glm::cross(
                    m1.nodesPredictedPos[t.out[1]] - p2,
                    m1.nodesPredictedPos[t.out[2]] - p2
                ));

                for (const unsigned int o : m2.tetras[tetra].out) {
                    const glm::vec3 p = m2.nodesPos[o];

                    if (glm::dot(p - p2, n2) < 0) {
                        const float dist = distanceToPlaneAlongDirection(p, -n1, p2, n2);
                        m2.nodesPredictedPos[o] += -n1 * dist;
                    }
                }
            }
        }
    }

    #pragma omp critical
    {
        m1.collided.insert(m1.collided.end(), local_collided.begin(), local_collided.end());
        m1.collidedNor.insert(m1.collidedNor.end(), local_collidedNor.begin(), local_collidedNor.end());
    }
}

    updateBVH(m2);
}

void handleCollision(Model &model) {
    model.collided.clear();
    model.collidedNor.clear();
    for (Model& m : models) {
        if (&m != &model && intersectBB(m.bb, model.bb)) {
            if (m.type == 0) {
                handleCollisionSS(model, m);
                updateBVH(model);
            }
            else {
                for (size_t node = 0; node < model.nodesPos.size(); node++) {
                    handleCollisionPM(model, node, m);
                }
            }
        }
    }
}


bool resolveCubeCollision(Model &model, const glm::vec3& point, const Model& m) {
    if (point.x > m.bb.minX && point.x < m.bb.maxX &&
        point.y > m.bb.minY && point.y < m.bb.maxY &&
        point.z > m.bb.minZ && point.z < m.bb.maxZ) {

        const glm::vec3 modelCenter = m.nodesPos.back();
        const float H = m.distH, V = m.distV, L = m.distL;

        const float dx = std::min(modelCenter.x + H - point.x, point.x - (modelCenter.x - H));
        const float dy = std::min(modelCenter.y + V - point.y, point.y - (modelCenter.y - V));
        const float dz = std::min(modelCenter.z + L - point.z, point.z - (modelCenter.z - L));

        float pen = dx;
        glm::vec3 normal(1.0f, 0.0f, 0.0f);

        if (dy < pen) {
            pen = dy;
            normal = glm::vec3(0.0f, 1.0f, 0.0f);
        }
        if (dz < pen) {
            pen = dz;
            normal = glm::vec3(0.0f, 0.0f, 1.0f);
        }

        if (glm::dot(point - modelCenter, normal) < 0.0f)
            normal = -normal;

        const glm::vec3 correctedPos = point + normal * pen;

        const unsigned int center = model.nodesPos.size() - 1;
        model.nodesPredictedPos[center] += correctedPos - point;
        model.collided.push_back(center);
        model.collidedNor.push_back(normal);

        return true;
        }
    return false;
}


void handleCollisionBM(Model &model, const Model &m) {
    constexpr int soft = 0;
    constexpr int axisAlignedBox = 1;
    constexpr int sphere = 2;

    const unsigned int center = model.nodesPos.size() - 1;

    if (m.type == axisAlignedBox) {
        for (const auto& point : model.nodesPredictedPos) {
            if (resolveCubeCollision(model, point, m)) {
                return;
            }
        }
    }

    if (m.type == sphere) {
        const glm::vec3 sphereCenter = m.nodesPos.back();
        const float radius = m.distH;

        for (const auto& point : model.nodesPredictedPos) {
            const glm::vec3 delta = point - sphereCenter;
            const float len = glm::length(delta);

            if (len < radius)
            {
                const glm::vec3 normal = (len > 1e-6f) ? delta / len : glm::vec3(0.0f, 1.0f, 0.0f);
                const float pen = radius - len;

                model.nodesPredictedPos[center] += normal * pen;;
                model.collided.push_back(center);
                model.collidedNor.push_back(normal);
            }
        }
    }

    if (m.type == soft) {
        for (const auto& point : model.nodesPredictedPos) {
            if (point.x > m.bb.minX && point.x < m.bb.maxX &&
                point.y > m.bb.minY && point.y < m.bb.maxY &&
                point.z > m.bb.minZ && point.z < m.bb.maxZ) {
                model.nodesPredictedPos[center] = model.nodesPos[center];
            }
        }
    }
}


void handleCollisionSM(Model &model, const Model &m) {
    const glm::vec3 delta = m.nodesPos[m.nodesPos.size() - 1] - model.nodesPredictedPos[model.nodesPos.size() - 1];
    const glm::vec3 dir = glm::normalize(delta);

    glm::vec3 point = model.nodesPredictedPos[model.nodesPos.size() - 1] + dir * model.distH;

    constexpr int soft = 0;
    constexpr int axisAlignedBox = 1;
    constexpr int sphere = 2;

    const int center = model.nodesPos.size() - 1;

    if (m.type == axisAlignedBox) {

        resolveCubeCollision(model, point, m);

        return;
    }

    if (model.type == sphere) {
        glm::vec3 deltaPoint = point - m.nodesPos[m.nodesPos.size() - 1];
        float radius = m.distH;
        float len = glm::length(deltaPoint);

        if (len < radius) {
            const glm::vec3 normal = (len > 1e-6f) ? deltaPoint / len : glm::vec3(0.0f, 1.0f, 0.0f);
            const float pen   = radius - len;

            model.nodesPredictedPos[center] += normal * pen;;
            model.collided.push_back(center);
            model.collidedNor.push_back(normal);
        }
    }

    if (m.type == soft) {
        const glm::vec3 modelCenter = glm::vec3((m.bb.maxX + m.bb.minX) / 2, (m.bb.maxY + m.bb.minY) / 2, (m.bb.maxZ + m.bb.minZ) / 2);
        point = model.nodesPredictedPos[model.nodesPos.size() - 1] + (modelCenter - model.nodesPredictedPos[model.nodesPos.size() - 1]) * model.distH;

        if (point.x > m.bb.minX && point.x < m.bb.maxX &&
            point.y > m.bb.minY && point.y < m.bb.maxY &&
            point.z > m.bb.minZ && point.z < m.bb.maxZ) {
            model.nodesPredictedPos[center] = model.nodesPos[center];
        }
    }
}

float facesAngle(const glm::vec3 Nl, const glm::vec3 Nr, const glm::vec3 Em, float *ArcCosSign = nullptr) {
    float result;
    const float cosAngle = glm::dot(Nl, Nr);

    if (glm::dot(cross(Nl,Nr), Em) < 0.0f) {
        result = M_PI * 2.0 - acos(cosAngle);
        *ArcCosSign = -1.0f;
    }
    else {
        result = acos(cosAngle);
        *ArcCosSign = 1.0f;
    }
    return result;
}


void updateSoftNodes(const float dt, Model& model) {
    const float dt_sub = dt / model.subStep;
    const vector<unsigned int>& indices = model.faces;

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
        for (size_t i = 0; i < model.nodesPos.size(); ++i) {
            model.nodesVel[i] += dt_sub * gravity;
            model.nodesPredictedPos[i] = model.nodesPos[i] + dt_sub * model.nodesVel[i];
        }

        // === SPRING CONSTRAINTS ===
        auto t0 = chrono::high_resolution_clock::now();

        auto& springsMap = model.springsMap;
        const int n = static_cast<int>(springsMap.size());

        #pragma omp parallel for
        for (int idx = 0; idx < n; idx++) {

            const auto it = std::next(springsMap.begin(), idx);
            const auto& springIndices = it->second;

            for (unsigned int s : springIndices) {
                Spring& spring = model.springs[s];

                const glm::vec3 delta = model.nodesPredictedPos[spring.A] - model.nodesPredictedPos[spring.B];

                const float lenSq = glm::dot(delta, delta);
                if (lenSq < 1e-12f) continue;

                const float len = sqrt(lenSq);
                const float C = len - spring.restLength;

                const float w1 = 1.0f / model.nodesMass[spring.A];
                const float w2 = 1.0f / model.nodesMass[spring.B];
                const float wSum = w1 + w2;

                const float alpha = spring.compliance / (dt_sub * dt_sub);
                const float deltaLambda = (-C - alpha * spring.lambda) / (wSum + alpha);

                const glm::vec3 dir = delta / len;

                model.nodesPredictedPos[spring.A] += w1 * dir * deltaLambda;
                model.nodesPredictedPos[spring.B] -= w2 * dir * deltaLambda;

                spring.lambda += deltaLambda;
            }
        }

        auto t1 = std::chrono::high_resolution_clock::now();

        springTime += chrono::duration<double, std::milli>(t1 - t0).count();
        springFrame++;

        t0 = chrono::high_resolution_clock::now();
        if (!model.tetra) {
            // === VOLUME CONSTRAINT ===
            const float currentVolume = calculateVolume(model.nodesPredictedPos, model.faces);
            const float C = currentVolume - model.volume;

            if (fabs(C) > 1e-6f) {
                vector<glm::vec3> jJ(model.nodesPos.size(), glm::vec3(0.0f));

                for (int i = 0; i < indices.size(); i += 3) {
                    const unsigned int I0 = indices[i];
                    const unsigned int I1 = indices[i + 1];
                    const unsigned int I2 = indices[i + 2];

                    const glm::vec3 p0 = model.nodesPredictedPos[I0];
                    const glm::vec3 p1 = model.nodesPredictedPos[I1];
                    const glm::vec3 p2 = model.nodesPredictedPos[I2];

                    jJ[I0] += glm::cross(p1, p2) / 6.0f;
                    jJ[I1] += glm::cross(p2, p0) / 6.0f;
                    jJ[I2] += glm::cross(p0, p1) / 6.0f;
                }

                const float weirdAlpha = model.volumeCompliance / (dt_sub * dt_sub);
                float denum = weirdAlpha;

                for (int i = 0; i < model.nodesPos.size(); i++) {
                    denum += (1.0f/model.nodesMass[i]) * glm::dot(jJ[i], jJ[i]);
                }

                if (fabs(denum) > 1e-6f) {
                    const float deltaLambda = (-C - weirdAlpha*(model.lambda)) / denum;

                    for (int i = 0; i < model.nodesPos.size(); i++) {
                        model.nodesPredictedPos[i] += ((1.0f/model.nodesMass[i])*deltaLambda) * jJ[i];
                    }

                    model.lambda += deltaLambda;
                }
            }

            // === BENDING ===
            for (AngledSpring& angledSpring : model.angledSprings) {
                const unsigned int N0 = angledSpring.edge.x;
                const unsigned int N1 = angledSpring.edge.y;
                const unsigned int N2 = angledSpring.A;
                const unsigned  N3 = angledSpring.B;

                const glm::vec3 P0 = model.nodesPredictedPos[N0];
                const glm::vec3 P1 = model.nodesPredictedPos[N1];
                const glm::vec3 P2 = model.nodesPredictedPos[N2];
                const glm::vec3 P3 = model.nodesPredictedPos[N3];

                const glm::vec3 Em = P1 - P0;
                const glm::vec3 El = P2 - P0;
                const glm::vec3 Er = P3 - P0;

                const glm::vec3 Nl = glm::cross(El, Em);
                const glm::vec3 Nr = glm::cross(Er, Em);

                const float Ll = glm::length(Nl);
                const float Lr = glm::length(Nr);

                if (fabs(Ll) > 1e-6f && fabs(Lr) > 1e-6f) {
                    const float cosAngle = calculateAngle(Nl, Nr);
                    float ArcCosSign = 1.0f;
                    const float currentAngle = facesAngle(Nl, Nr, Em, &ArcCosSign);
                    const float deltaAngle = currentAngle - angledSpring.restAngle;

                    if (fabs(deltaAngle) > 1e-6f) {
                        const glm::vec3 JP1 = ((glm::cross(Er, Nl) + cosAngle*glm::cross(Nr, Er)))/Lr + (glm::cross(El, Nr) + cosAngle*glm::cross(Nl, El));
                        const glm::vec3 JP2 = (glm::cross(Nr, Em) - cosAngle*glm::cross(Nl, Em))/Ll;
                       const  glm::vec3 JP3 = (glm::cross(Nl, Em) - cosAngle*glm::cross(Nr, Em))/Lr;
                        const glm::vec3 JP0 = -JP1 - JP2 - JP3;

                        const float RadicanDenomF = 1.0f - cosAngle*cosAngle;
                        const float DenomF = -ArcCosSign*sqrt(RadicanDenomF);


                        const float W0 = 1.0f / model.nodesMass[N0];
                        const float W1 = 1.0f / model.nodesMass[N1];
                        const float W2 = 1.0f / model.nodesMass[N2];
                        const float W3 = 1.0f / model.nodesMass[N3];

                        const float weirdAlpha = angledSpring.compliance / (dt_sub * dt_sub);

                        const float Denom = W0 * glm::dot(JP0, JP0) + W1 * glm::dot(JP1, JP1) + W2 * glm::dot(JP2, JP2) + W3 * glm::dot(JP3, JP3) + weirdAlpha*RadicanDenomF;

                        if (fabs(Denom) > 1e-6f) {
                            const float deltaLambda = (-deltaAngle - weirdAlpha*(angledSpring.lambda))/Denom;
                            model.nodesPredictedPos[N0] += (W0*(DenomF*DenomF)*deltaLambda)*JP0;
                            model.nodesPredictedPos[N1] += (W1*(DenomF*DenomF)*deltaLambda)*JP1;
                            model.nodesPredictedPos[N2] += (W2*(DenomF*DenomF)*deltaLambda)*JP2;
                            model.nodesPredictedPos[N3] += (W3*(DenomF*DenomF)*deltaLambda)*JP3;

                            angledSpring.lambda += deltaLambda*RadicanDenomF;
                        }
                    }
                }
            }
        }
        else {
            // === VOLUME CONSTRAINT ===
            #pragma omp parallel for
            for (int i = 0; i < model.tetras.size(); i++) {
                Tetra& tetra = model.tetras[i];
                const float currentVolume = calculateTetraVolume(tetra, model.nodesPredictedPos);
                const float C = currentVolume - tetra.volume;

                if (fabs(C) > 1e-6f) {
                    // Get indices of the tetrahedron's vertices
                    const unsigned int I0 = tetra.out[0];
                    const unsigned int I1 = tetra.out[1];
                    const unsigned int I2 = tetra.out[2];
                    const unsigned int I3 = tetra.in;

                    const glm::vec3 p0 = model.nodesPredictedPos[I0];
                    const glm::vec3 p1 = model.nodesPredictedPos[I1];
                    const glm::vec3 p2 = model.nodesPredictedPos[I2];
                    const glm::vec3 p3 = model.nodesPredictedPos[I3];

                    const glm::vec3 j0 = glm::cross(p1 - p2, p3 - p2) / 6.0f;
                    const glm::vec3 j1 = glm::cross(p2 - p0, p3 - p0) / 6.0f;
                    const glm::vec3 j2 = glm::cross(p0 - p1, p3 - p1) / 6.0f;
                    const glm::vec3 j3 = glm::cross(p1 - p0, p2 - p0) / 6.0f;

                    const float weirdAlpha = model.volumeCompliance / (dt_sub * dt_sub);
                    float denum = weirdAlpha;

                    // Compute denominator
                    denum += (1.0f / model.nodesMass[I0]) * glm::dot(j0, j0);
                    denum += (1.0f / model.nodesMass[I1]) * glm::dot(j1, j1);
                    denum += (1.0f / model.nodesMass[I2]) * glm::dot(j2, j2);
                    denum += (1.0f / model.nodesMass[I3]) * glm::dot(j3, j3);

                    if (fabs(denum) > 1e-6f) {
                        const float deltaLambda = (-C - weirdAlpha * model.lambda) / denum;

                        model.nodesPredictedPos[I0] += (1.0f / model.nodesMass[I0]) * deltaLambda * j0;
                        model.nodesPredictedPos[I1] += (1.0f / model.nodesMass[I1]) * deltaLambda * j1;
                        model.nodesPredictedPos[I2] += (1.0f / model.nodesMass[I2]) * deltaLambda * j2;
                        model.nodesPredictedPos[I3] += (1.0f / model.nodesMass[I3]) * deltaLambda * j3;

                        tetra.lambda += deltaLambda;
                    }
                }
            }
        }
        t1 = chrono::high_resolution_clock::now();
        volumeTime += chrono::duration<double, std::milli>(t1 - t0).count();
        volumeFrame++;

        t0 = chrono::high_resolution_clock::now();
        // === COLLISION ===
        for (size_t i = 0; i < model.nodesPos.size(); ++i) {
            model.nodesVel[i] = (model.nodesPredictedPos[i] - model.nodesPos[i]) / dt_sub;
        }

        handleCollision(model);

        for (size_t i = 0; i < model.collided.size(); ++i) {
            const unsigned int idx = model.collided[i];
            const glm::vec3 velocity = model.nodesVel[idx];
            const glm::vec3 normal = model.collidedNor[i];

            // Decompose velocity into normal and tangential components
            const float velocity_normal_magnitude = glm::dot(velocity, normal);
            const glm::vec3 velocity_normal = velocity_normal_magnitude * normal;
            const glm::vec3 velocity_tangent = velocity - velocity_normal;

            // Reflect and dampen the normal component
            const glm::vec3 reflected = -model.bounciness * velocity_normal;

            // Apply friction to the tangential component
            const glm::vec3 frictioned = velocity_tangent * friction;

            // Combine
            model.nodesVel[idx] = reflected + frictioned;
        }

        for (size_t i = 0; i < model.nodesPos.size(); ++i) {
            model.nodesPos[i] = model.nodesPredictedPos[i];
            model.nodesVel[i] *= 0.9999f;
        }
        t1 = chrono::high_resolution_clock::now();
        collisionTime += chrono::duration<double, std::milli>(t1 - t0).count();
        collisionFrame++;

    }

    auto t0 = chrono::high_resolution_clock::now();

    computeVertexNormals(model.nodesNor, model.nodesPos, model.faces);

    auto t1 = chrono::high_resolution_clock::now();
    normalTime += chrono::duration<double, std::milli>(t1 - t0).count();
    normalFrame++;

    updateGPUpositionsAndNormals(model);

    createBB(model);
}


void updateRigidNodes(const float dt, Model& model) {
    model.nodesVel[model.nodesPos.size() - 1] += dt * gravity;

    for (size_t i = 0; i < model.nodesPos.size(); ++i) {
        model.nodesPredictedPos[i] = model.nodesPos[i] + model.nodesVel[model.nodesPos.size() - 1] * dt;
    }

    for (Model& m : models) {
        if (&m != &model) {
            if (model.type == 1) {
                handleCollisionBM(model, m);
            }
            else {
                if (model.type == 2) {
                    handleCollisionSM(model, m);
                }
            }
        }
    }

    const glm::vec3 deltaPos = (model.nodesPredictedPos[model.nodesPos.size() - 1] - model.nodesPos[model.nodesPos.size() - 1]);

    if (model.collided.empty()) {
        model.nodesVel[model.nodesPos.size() - 1] = deltaPos / dt;
    }
    else {
        for (size_t i = 0; i < model.collided.size(); ++i) {
            const unsigned int idx = model.collided[i];
            const glm::vec3 velocity = model.nodesVel[idx];
            const glm::vec3 normal = model.collidedNor[i];

            // Decompose velocity into normal and tangential components
            const float velocity_normal_magnitude = glm::dot(velocity, normal);
            const glm::vec3 velocity_normal = velocity_normal_magnitude * normal;
            const glm::vec3 velocity_tangent = velocity - velocity_normal;

            // Reflect and dampen the normal component
            const glm::vec3 reflected = -model.bounciness * velocity_normal;

            // Apply friction to the tangential component
            const glm::vec3 frictioned = velocity_tangent * friction;

            // Combine
            model.nodesVel[idx] = reflected + frictioned;
        }
    }

    for (auto & nodesPo : model.nodesPos) {
        nodesPo += deltaPos;
    }

    createBB(model);

    updateGPUpositionsAndNormals(model);
}

void updateNodes(const float dt, Model& model) {
    model.collided.clear();
    model.collidedNor.clear();
    if (model.type == 0) {
        updateSoftNodes(dt, model);
    }
    else {
        updateRigidNodes(dt, model);
    }
}

void drawBB(const BB& bb) {
    glBegin(GL_LINES);

    // Bottom face
    glVertex3f(bb.minX, bb.minY, bb.minZ);
    glVertex3f(bb.maxX, bb.minY, bb.minZ);

    glVertex3f(bb.maxX, bb.minY, bb.minZ);
    glVertex3f(bb.maxX, bb.maxY, bb.minZ);

    glVertex3f(bb.maxX, bb.maxY, bb.minZ);
    glVertex3f(bb.minX, bb.maxY, bb.minZ);

    glVertex3f(bb.minX, bb.maxY, bb.minZ);
    glVertex3f(bb.minX, bb.minY, bb.minZ);

    // Top face
    glVertex3f(bb.minX, bb.minY, bb.maxZ);
    glVertex3f(bb.maxX, bb.minY, bb.maxZ);

    glVertex3f(bb.maxX, bb.minY, bb.maxZ);
    glVertex3f(bb.maxX, bb.maxY, bb.maxZ);

    glVertex3f(bb.maxX, bb.maxY, bb.maxZ);
    glVertex3f(bb.minX, bb.maxY, bb.maxZ);

    glVertex3f(bb.minX, bb.maxY, bb.maxZ);
    glVertex3f(bb.minX, bb.minY, bb.maxZ);

    // Vertical lines
    glVertex3f(bb.minX, bb.minY, bb.minZ);
    glVertex3f(bb.minX, bb.minY, bb.maxZ);

    glVertex3f(bb.maxX, bb.minY, bb.minZ);
    glVertex3f(bb.maxX, bb.minY, bb.maxZ);

    glVertex3f(bb.maxX, bb.maxY, bb.minZ);
    glVertex3f(bb.maxX, bb.maxY, bb.maxZ);

    glVertex3f(bb.minX, bb.maxY, bb.minZ);
    glVertex3f(bb.minX, bb.maxY, bb.maxZ);

    glEnd();
}


void drawModel(const Model &model)
{
    const GLModel &g = model.glModel;
    if (!g.vao) return;

    if (model.wireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    }


    glBindVertexArray(g.vao);
    glDrawElements(GL_TRIANGLES, g.indexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    drawBB(model.bb);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

}



void changeSize(const int w, int h) {
    if (h == 0) h = 1;

    const float ratio = static_cast<float>(w) / h;

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glViewport(0, 0, w, h);
    gluPerspective(45.0f, ratio, 1.0f, 1000.0f);
    glMatrixMode(GL_MODELVIEW);
}

void renderScene() {
    char s[64];

    currentTime = glutGet(GLUT_ELAPSED_TIME);

    float dt = currentTime - previousTime;
    previousTime = currentTime;

    dt /= 1000.0f;

    constexpr float MAX_DT = 1.0f / 30.0f; // Max of 33ms (like 30fps)
    dt = std::min(dt, MAX_DT);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(shaderProgram);

    if (!paused && dt > 0.00001f) {
        for (int i = 0; i < updateModels.size(); ++i) {
            updateNodes(dt, models[i]);
        }
    }

    auto model = glm::mat4(1.0f);
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
    const int time = glutGet(GLUT_ELAPSED_TIME);
    if (time - timebase > 1000) {
        const double fps = frame * 1000.0 / (time - timebase);
        timebase = time;
        frame = 0;

        sprintf_s(s, "FPS: %f6.2", fps);
        glutSetWindowTitle(s);

        std::cout << "Spring step took: "
          << springTime/springFrame
          << " ms" << std::endl;

        springFrame = 0;
        springTime = 0;

        std::cout << "Volume step took: "
          << volumeTime/volumeFrame
          << " ms" << std::endl;

        volumeFrame = 0;
        volumeTime = 0;

        std::cout << "Collision step took: "
          << collisionTime/collisionFrame
          << " ms" << std::endl;

        collisionFrame = 0;
        collisionTime = 0;

        std::cout << "normal step took: "
          << normalTime/normalFrame
          << " ms" << std::endl;

        normalFrame = 0;
        normalTime = 0;
    }

    glutSwapBuffers();
}

void keyManage(const unsigned char key, int x, int y) {
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
            if (!paused) {
                previousTime = glutGet(GLUT_ELAPSED_TIME);
                currentTime = glutGet(GLUT_ELAPSED_TIME);
            }
            break;
        default:
            break;
    }

    glutPostRedisplay();
}

GLuint compileShader(const char* path, const GLenum type) {
    const ifstream file(path);
    if (!file.is_open()) {
        cerr << "Failed to open shader file: " << path << endl;
        return 0;
    }

    stringstream buffer;
    buffer << file.rdbuf();

    const string src = buffer.str();
    const char* csrc = src.c_str();

    const GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &csrc, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader compilation failed for " << path << ":\n" << infoLog << std::endl;
        return 0;
    }
    return shader;
}


GLuint createShaderProgram(const char* vertPath, const char* fragPath) {
    const GLuint vert = compileShader(vertPath, GL_VERTEX_SHADER);
    if (vert == 0) return 0;

    const GLuint frag = compileShader(fragPath, GL_FRAGMENT_SHADER);
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
        glGetProgramInfoLog(prog, 512, nullptr, infoLog);
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

    loadLevel("levels/level0-0.lvl");

    previousTime = glutGet(GLUT_ELAPSED_TIME);
    currentTime = glutGet(GLUT_ELAPSED_TIME);

    glutDisplayFunc(renderScene);
    glutIdleFunc(renderScene);
    glutReshapeFunc(changeSize);
    glutKeyboardFunc(keyManage);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glutMainLoop();


    return 0;
}