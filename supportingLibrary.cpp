#include "supportingLibrary.h"

// Calculates the Volume of any model using Positions
float calculateVolume(const vector<glm::vec3> &nodesPos, const vector<unsigned int> &faces) {
    float volume = 0.0f;

    for (int i = 0; i < faces.size(); i+=3) {
        glm::vec3 v1 = nodesPos[faces[i]];
        glm::vec3 v2 = nodesPos[faces[i+1]];
        glm::vec3 v3 = nodesPos[faces[i+2]];

        volume += glm::dot(v1, glm::cross(v2,v3))/6;
    }
    return volume;
}

// Calculates the Volume of a Tetrahedron using Positions
float calculateTetraVolume(const Tetra &tetra, const vector<glm::vec3> &nodesPos) {
    const glm::vec3& p0 = nodesPos[tetra.out[0]];
    const glm::vec3& p1 = nodesPos[tetra.out[1]];
    const glm::vec3& p2 = nodesPos[tetra.out[2]];
    const glm::vec3& p3 = nodesPos[tetra.in];

    return glm::dot(glm::cross(p1 - p0, p2 - p0), p3 - p0) / 6.0f;
}

// Calculates Current Angle between 2 vectors
float calculateAngle(const glm::vec3 &vec1, const glm::vec3 &vec2) {
    glm::vec3 v1 = glm::normalize(vec1);
    glm::vec3 v2 = glm::normalize(vec2);

    float d1 = glm::length(v1);
    float d2 = glm::length(v2);

    if (d1 < 1e-6f || d2 < 1e-6f) return 0.0f;

    float dot = glm::dot(v1, v2) / (d1 * d2);

    // Clamp dot product to valid range [-1, 1] to avoid NaNs in acos
    dot = glm::clamp(dot, -1.0f, 1.0f);

    return dot;
}

// Returns the Shared Edge between 2 Triangles
glm::vec2 getEdge(const vector<glm::vec2> &edgesA, const vector<glm::vec2> &edgesB) {
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
    for (glm::vec3 &node : model.nodesPos) {
        bb.minX = std::min(node.x, bb.minX);
        bb.minY = std::min(node.y, bb.minY);
        bb.minZ = std::min(node.z, bb.minZ);
        bb.maxX = std::max(node.x, bb.maxX);
        bb.maxY = std::max(node.y, bb.maxY);
        bb.maxZ = std::max(node.z, bb.maxZ);
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

BB createBBPoints(const glm::vec3 p1, const glm::vec3 p2, const glm::vec3 p3) {
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

// Creates a Bounding Volume for a Tetrahedron
void createBBTetra(Tetra &tetra, const vector<glm::vec3> &nodesPos) {
    BB bb;
    bb.minX = nodesPos[tetra.in].x;
    bb.maxX = nodesPos[tetra.in].x;
    bb.minY = nodesPos[tetra.in].y;
    bb.maxX = nodesPos[tetra.in].y;
    bb.minZ = nodesPos[tetra.in].z;
    bb.maxX = nodesPos[tetra.in].z;

    for (const unsigned int o : tetra.out) {
        bb.minX = std::min(bb.minX, nodesPos[o].x);
        bb.minY = std::min(bb.minY, nodesPos[o].y);
        bb.minZ = std::min(bb.minZ, nodesPos[o].z);
        bb.maxX = std::max(bb.maxX, nodesPos[o].x);
        bb.maxY = std::max(bb.maxY, nodesPos[o].y);
        bb.maxZ = std::max(bb.maxZ, nodesPos[o].z);
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

// Adds 2 Bounding Boxes together
BB addBB(const BB &bb1, const BB &bb2) {
    BB bb;
    bb.minX = std::min(bb1.minX, bb2.minX);
    bb.minY = std::min(bb1.minY, bb2.minY);
    bb.minZ = std::min(bb1.minZ, bb2.minZ);
    bb.maxX = std::max(bb1.maxX, bb2.maxX);
    bb.maxY = std::max(bb1.maxY, bb2.maxY);
    bb.maxZ = std::max(bb1.maxZ, bb2.maxZ);
    return bb;
}

bool insideBB(const BB &bb, glm::vec3 p) {
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

// Creates a Bouding Volume Hierarchy of Tetrahedrons Bounding Boxes for a Model
void createModelVBHTetra(Model &model) {
    vector<BVH> bvhsTemp;
    vector<BVH> bvhs;

    int k = 0;

    // Create leaf BVHs
    for (int i = 0; i < model.tetras.size(); i++) {
        BVH bvh;
        createBBTetra(model.tetras[i], model.nodesPos);
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

void updateBVHLeafs (Model &model) {
    for (int i = 0; i < model.tetras.size(); i++) {
        createBBTetra(model.tetras[model.bvhs[i].tetra], model.nodesPos);
        model.bvhs[i].bb = model.tetras[model.bvhs[i].tetra].bb;
    }
}

void updateBVH (Model &model) {
    updateBVHLeafs(model);
    for (unsigned int i = model.tetras.size(); i < model.bvhs.size(); i++) {
        model.bvhs[i].bb = addBB(model.bvhs[model.bvhs[i].left].bb, model.bvhs[model.bvhs[i].right].bb);
    }
}

// Calculates and Saves the Normals of each Node for use in Light
void computeVertexNormals(vector<glm::vec3> &normals, const vector<glm::vec3> &nodesPos, const vector<unsigned int> &faces) {
    // Initialize all normals to zero
    normals = vector<glm::vec3> (nodesPos.size(), glm::vec3(0.0f));
    vector<int> division(nodesPos.size(), 0);

    for (size_t i = 0; i < faces.size(); i += 3) {
        const unsigned int i0 = faces[i];
        const unsigned int i1 = faces[i + 1];
        const unsigned int i2 = faces[i + 2];

        glm::vec3 v0 = nodesPos[i0];
        glm::vec3 v1 = nodesPos[i1];
        glm::vec3 v2 = nodesPos[i2];

        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;
        glm::vec3 faceNormal = glm::cross(edge1, edge2);
        if (glm::length(faceNormal) > 0.00001f) {
            faceNormal = glm::normalize(faceNormal);
        } else {
            continue; // skip degenerate face
        }
        ;

        normals[i0] += faceNormal;
        division[i0] += 1;
        normals[i1] += faceNormal;
        division[i1] += 1;
        normals[i2] += faceNormal;
        division[i2] += 1;
    }

    for (size_t i = 0; i < normals.size(); ++i) {
        if (division[i] != 0) {
            normals[i] /= division[i];
        }
        if (glm::length(normals[i]) > 0.00001f) {
            normals[i] = glm::normalize(normals[i]);
        } else {
            normals[i] = glm::vec3(0.0f); // fallback value
        }

    }
}