#define GLM_ENABLE_EXPERIMENTAL
#define _USE_MATH_DEFINES

#include <iostream>
#include <filesystem>
#include "supportingLibrary.h"
#include "tetgen.h"

using namespace std;
namespace fs = filesystem;

constexpr float GOLDEN_RATIO = 1.618f;
constexpr glm::vec3 ORIGIN = glm::vec3 (0.0f);
constexpr float EPSILON = 1e-6f;



void calculateTetras(Model &model, float compliance, float centerCompliance, float mass, int surfaceType) {
        vector<Tetra> tetras;
        tetgenio in, out;
        in.numberofpoints = model.nodesPos.size();
        in.pointlist = new REAL[in.numberofpoints * 3];

        for (int i = 0; i < model.nodesPos.size(); ++i) {
            in.pointlist[i * 3 + 0] = model.nodesPos[i].x;
            in.pointlist[i * 3 + 1] = model.nodesPos[i].y;
            in.pointlist[i * 3 + 2] = model.nodesPos[i].z;
        }

        in.numberoffacets = model.faces.size() / 3;
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
            p.vertexlist[0] = model.faces[i * 3 + 0];
            p.vertexlist[1] = model.faces[i * 3 + 1];
            p.vertexlist[2] = model.faces[i * 3 + 2];

            in.facetmarkerlist[i] = 0;
        }

        in.numberofregions = 1;
        in.regionlist = new REAL[4];
        in.regionlist[0] = 0;
        in.regionlist[1] = 0;
        in.regionlist[2] = 0;
        in.regionlist[3] = 1;

        tetgenbehavior behavior;
        behavior.parse_commandline(const_cast<char *>("pa0.1Y"));
        tetrahedralize(&behavior, &in, &out);

        model.nodesMass.resize(out.numberofpoints);
        model.nodesPos.resize(out.numberofpoints);
        for (int i = 0; i < out.numberofpoints; i++) {
            model.nodesPos[i] = glm::vec3(out.pointlist[i*3], out.pointlist[i*3 + 1], out.pointlist[i*3 + 2]);
            model.nodesMass[i] = mass;
        }

        vector<Spring> springs;

        for (int i = 0; i < out.numberoftetrahedra; ++i) {
            int a = out.tetrahedronlist[i * 4 + 0];
            int b = out.tetrahedronlist[i * 4 + 1];
            int c = out.tetrahedronlist[i * 4 + 2];
            int d = out.tetrahedronlist[i * 4 + 3];

            vector<int> points {a,b,c,d};

            Tetra t;
            vector<unsigned int> outT;

            if (surfaceType == 2) {
                for (auto& p : points) {
                    float dist = glm::length(model.nodesPos[p]);
                    if ( model.distH - dist > 0.001f) {
                        t.in = p;
                    }
                    else {
                        outT.push_back(p);
                    }
                }
            }
            if (surfaceType == 1) {
                for (auto& p : points) {
                    const glm::vec3& pos = model.nodesPos[p];

                    bool insideX = (pos.x > -model.distH + EPSILON) && (pos.x < model.distH - EPSILON);
                    bool insideY = (pos.y > -model.distV + EPSILON) && (pos.y < model.distV - EPSILON);
                    bool insideZ = (pos.z > -model.distL + EPSILON) && (pos.z < model.distL - EPSILON);

                    if (insideX && insideY && insideZ) {
                        t.in = p;
                    } else {
                        outT.push_back(p);
                    }
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
            t.volume = calculateTetraVolume(t, model.nodesPos);
            t.surface = surface;


            auto addSpring = [&](int i, int j) {
                Spring s;
                s.A = i;
                s.B = j;
                s.restLength = glm::length(model.nodesPos[i] - model.nodesPos[j]);
                if (glm::length(model.nodesPos[i]) < model.distH || glm::length(model.nodesPos[j]) < model.distH) {
                    s.compliance = centerCompliance;
                    s.in = true;
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
        model.springs = springs;

        delete[] in.trifacelist;
}

void calculateSprings(Model &model, float compliance) {
    vector<Spring> springs;

    for (int i = 0; i < model.faces.size(); i += 3) {
        int A = model.faces[i];
        int B = model.faces[i + 1];
        int C = model.faces[i + 2];

        Spring s1;
        Spring s2;
        Spring s3;

        s1.A = A;
        s1.B = B;
        s1.restLength = glm::length(model.nodesPos[A] - model.nodesPos[B]);
        s1.compliance = compliance;

        s2.A = B;
        s2.B = C;
        s2.restLength = glm::length(model.nodesPos[B] - model.nodesPos[C]);
        s2.compliance = compliance;

        s3.A = C;
        s3.B = A;
        s3.restLength = glm::length(model.nodesPos[C] - model.nodesPos[A]);
        s3.compliance = compliance;

        springs.push_back(s1);
        springs.push_back(s2);
        springs.push_back(s3);
    }

    model.springs = springs;
}

void calculateAngledSprings(Model &model, float angleStiffness) {
    vector<AngledSpring> angledSprings;

        for (int i = 0; i < model.faces.size() - 3; i += 3) {
            int A = model.faces[i];
            int B = model.faces[i + 1];
            int C = model.faces[i + 2];

            for (int j = i + 3; j < model.faces.size(); j += 3) {
                int D = model.faces[j];
                int E = model.faces[j + 1];
                int F = model.faces[j + 2];

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
                    glm::vec3 v1 = glm::cross((model.nodesPos[uniqueA] - model.nodesPos[edge.x]), (model.nodesPos[edge.y] - model.nodesPos[edge.x]));
                    glm::vec3 v2 = glm::cross((model.nodesPos[uniqueB] -model. nodesPos[edge.x]), (model.nodesPos[edge.y] - model.nodesPos[edge.x]));
                    s.restAngle = acos(calculateAngle(v1, v2));
                    angledSprings.push_back(s);
                }
            }
        }
        model.angledSprings = angledSprings;
}

void calculateSpringMap(Model &model) {
    unordered_map<int, vector<int>> springsMap;
    int i = 0;

    for (const Spring& s : model.springs) {
        springsMap[s.A].push_back(i);

        i++;
    }

    model.springsMap = springsMap;
}




// Makes a Soft Object with the shape of a IcoSphere
Model makeIcoSphere(float radius, int detail, float subStep = 1.0f, float mass = 1.0f, float bounciness = 0.0f, float volumeCompliance = 0.0f,
                    float compliance = 0.0f, float centerCompliance = 0.0f, float angleStiffness = 0.0f,
                    glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f), bool tetra = false, bool soft = false) {
    vector<float> nodesMass;
    vector<glm::vec3> nodesPos;
    std::unordered_map<glm::vec3, int, Vec3Hash, Vec3Equal> vertexMap;
    vector<int> faces;

    Model model;

    model.distV = model.distH = model.distL = radius;
    model.color = color;
    model.bounciness = bounciness;

    float a = sqrt(radius / (1 + GOLDEN_RATIO * GOLDEN_RATIO));
    float c = a * GOLDEN_RATIO;
    glm::vec3 pos;

    float x = 0.0f;
    float y = -c;
    float z = a;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 0;

    y = -c;
    z = -a;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 1;

    x = c;
    y = -a;
    z = 0.0f;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 2;

    x = -c;
    y = -a;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 3;

    x = a;
    y = 0.0f;
    z = c;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 4;

    x = a;
    z = -c;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 5;

    x = -a;
    z = -c;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 6;

    x = -a;
    z = c;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 7;

    x = c;
    y = a;
    z = 0.0f;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 8;

    x = -c;
    y = a;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 9;

    x = 0.0f;
    y = c;
    z = a;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 10;

    y = c;
    z = -a;
    pos = glm::normalize(glm::vec3(x, y, z)) * radius;
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);
    vertexMap[pos] = 11;

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
            glm::vec3 v1 = nodesPos[faces[j]];
            glm::vec3 v2 = nodesPos[faces[j+1]];
            glm::vec3 v3 = nodesPos[faces[j+2]];

            glm::vec3 p1;
            float m1 = mass;
            glm::vec3 midpoint1 = (v2 + v1) * 0.5f;
            p1 = glm::normalize(midpoint1) * radius;

            glm::vec3 p2;
            float m2 = mass;
            glm::vec3 midpoint2 = (v3 + v2) * 0.5f;
            p2 = glm::normalize(midpoint2) * radius;

            glm::vec3 p3;
            float m3 = mass;
            glm::vec3 midpoint3 = (v1 + v3) * 0.5f;
            p3 = glm::normalize(midpoint3) * radius;

            int i1, i2, i3;

            auto it = vertexMap.find(p1);
            if (it == vertexMap.end()) {
                i1 = nodesPos.size();
                nodesMass.push_back(m1);
                nodesPos.push_back(p1);
                vertexMap[p1] = i1;
            } else {
                i1 = it->second;
            }

            it = vertexMap.find(p2);
            if (it == vertexMap.end()) {
                i2 = nodesPos.size();
                nodesMass.push_back(m2);
                nodesPos.push_back(p2);
                vertexMap[p2] = i2;
            } else {
                i2 = it->second;
            }

            it = vertexMap.find(p3);
            if (it == vertexMap.end()) {
                i3 = nodesPos.size();
                nodesMass.push_back(m3);
                nodesPos.push_back(p3);
                vertexMap[p3] = i3;
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

    model.nodesMass = nodesMass;
    model.nodesPos = nodesPos;
    model.faces = vector<unsigned int>(faces.begin(), faces.end());


    if (soft) {
        model.type = 0;
        model.tetra = tetra;

        if (tetra == true) {  // Using Tetrahedrons (uses tetgen)
            int surfaceType = 2; // 2 = sphere
            calculateTetras(model, compliance, centerCompliance, mass, surfaceType);
        }
        else {
            calculateSprings(model, compliance);
            calculateAngledSprings(model, angleStiffness);
        }

        model.volumeCompliance = volumeCompliance;
        model.volume = calculateVolume(model.nodesPos, model.faces);
        model.subStep = subStep;
    }
    else {
        model.type = 2;
        glm::vec3 center = glm::vec3(0, 0, 0);
        model.nodesMass.push_back(mass);
        model.nodesPos.push_back(center);
    }

    calculateSpringMap(model);

    computeVertexNormals(model.nodesNor, model.nodesPos, model.faces);

    createBB(model);
    createModelVBHTetra(model);

    printf("Number of points %d Number of Faces %d Number of springs %d \n", static_cast<int>(model.nodesPos.size()), static_cast<int>(model.faces.size()), static_cast<int>(model.springs.size()));
    return model;
}

// Makes a Soft Object with the shape of a Globe Sphere
Model makeSphere(float radius, int slices, int stacks, float subStep = 1.0f, float mass = 1.0f, float bounciness = 0.0f, float volumeCompliance = 0.0f,
                    float compliance = 0.0f, float centerCompliance = 0.0f, float angleStiffness = 0.0f,
                    glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f), bool tetra = false, bool soft = false) {
    vector<float> nodesMass;
    vector<glm::vec3> nodesPos;
    unordered_map<glm::vec3, int, Vec3Hash, Vec3Equal> vertexMap;
    vector<int> faces;

    Model model;

    model.distV = model.distH = model.distL = radius;
    model.color = color;
    model.bounciness = bounciness;

    glm::vec3 pos;

    // Top vertex
    pos = glm::vec3(0, radius, 0);
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);

    // Generate middle stack vertices (excluding poles)
    for (int i = 1; i < stacks; ++i) {
        float phi = M_PI * i / stacks;
        float y = cos(phi);
        float r = sin(phi);

        for (int j = 0; j < slices; ++j) {
            float theta = 2 * M_PI * j / slices;
            float x = r * cos(theta);
            float z = r * sin(theta);
            pos = glm::vec3(x * radius, y * radius, z * radius);
            nodesMass.push_back(mass);
            nodesPos.push_back(pos);
        }
    }

    // Bottom vertex
    pos = glm::vec3(0, -radius, 0);
    nodesMass.push_back(mass);
    nodesPos.push_back(pos);

    int bottomIndex = static_cast<int>(nodesPos.size()) - 1;

    // index helper (1-based stack index because of shared top)
    auto idx = [&](const int stack, int slice) {
        slice = ((slice % slices) + slices) % slices;
        return 1 + (stack - 1) * slices + slice;
    };

    // Faces Generation

    // Top cap
    for (int j = 0; j < slices; ++j) {
        constexpr int topIndex = 0;
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

    model.nodesMass = nodesMass;
    model.nodesPos = nodesPos;
    model.faces = vector<unsigned int>(faces.begin(), faces.end());

    if (soft) {
        model.type = 0;
        model.tetra = tetra;

        if (tetra == true) {  // Using Tetrahedrons (uses tetgen)
            int surfaceType = 2; // 2 = sphere
            calculateTetras(model, compliance, centerCompliance, mass, surfaceType);
        }
        else {
            calculateSprings(model, compliance);
            calculateAngledSprings(model, angleStiffness);
        }

        model.volumeCompliance = volumeCompliance;
        model.volume = calculateVolume(model.nodesPos, model.faces);
        model.subStep = subStep;
    }
    else {
        model.type = 2;
        glm::vec3 center = glm::vec3(0, 0, 0);
        model.nodesMass.push_back(mass);
        model.nodesPos.push_back(center);
    }

    calculateSpringMap(model);

    computeVertexNormals(model.nodesNor, model.nodesPos, model.faces);

    createBB(model);
    createModelVBHTetra(model);

    printf("Number of points %d Number of Faces %d Number of springs %d \n", static_cast<int>(model.nodesPos.size()), static_cast<int>(model.faces.size()), static_cast<int>(model.springs.size()));
    return model;
}

// Makes a Object with the shape of a Box
Model makeBox(int slices, int stacks, float subStep, float mass = 1.0f, float bounciness = 0.0f, float volumeCompliance = 0.0f,
                float compliance = 0.0f, float centerCompliance = 0.0f , float angleStiffness = 0.0f, float H = 1, float V = 1, float L = 1,
                glm::vec3 color = glm::vec3(1.0f, 1.0f, 1.0f), bool tetra = false, bool soft = false) {
    vector<float> nodesMass;
    vector<glm::vec3> nodesPos;
    vector<int> faces;
    std::unordered_map<glm::vec3, int, Vec3Hash, Vec3Equal> vertexMap;

    Model model;

    model.distV = V, model.distH = H, model.distL = L;
    model.color = color;
    model.bounciness = bounciness;

    glm::vec3 pos;

    vector<glm::vec3> positions;
    vector<int> indexs;
    int index = 0;
    float x, y, z;

    // First Face
    z = -L;
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            x = -H + H*2/slices*j;
            if (pos = glm::vec3(x, y, z); vertexMap.find(pos) == vertexMap.end()) {
                vertexMap[pos] = index;
                nodesMass.push_back(mass);
                nodesPos.push_back(pos);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[pos]);
            }
        }
    }

    // Second Face
    x = H;
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            z = -L + L*2/slices*j;
            if (pos = glm::vec3(x, y, z); vertexMap.find(pos) == vertexMap.end()) {
                vertexMap[pos] = index;
                nodesMass.push_back(mass);
                nodesPos.push_back(pos);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[pos]);
            }
        }
    }


    // Third Face
    z = L;
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            x = H - H*2/slices*j;
            if (pos = glm::vec3(x, y, z); vertexMap.find(pos) == vertexMap.end()) {
                vertexMap[pos] = index;
                nodesMass.push_back(mass);
                nodesPos.push_back(pos);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[pos]);
            }
        }
    }

    // Fourth Face
    x = -H;
    for (int i = 0; i <= stacks; ++i) {
        y = -V + V*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            z = L - L*2/slices*j;
            if (pos = glm::vec3(x, y, z); vertexMap.find(pos) == vertexMap.end()) {
                vertexMap[pos] = index;
                nodesMass.push_back(mass);
                nodesPos.push_back(pos);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[pos]);
            }
        }
    }

    // Fith Face
    y = -V;
    for (int i = 0; i <= stacks; ++i) {
        x = -H + H*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            z = -L + L*2/slices*j;
            if (pos = glm::vec3(x, y, z); vertexMap.find(pos) == vertexMap.end()) {
                vertexMap[pos] = index;
                nodesMass.push_back(mass);
                nodesPos.push_back(pos);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[pos]);
            }
        }
    }

    // Sixth Face
    y = V;
    for (int i = 0; i <= stacks; ++i) {
        x = H - H*2/stacks*i;
        for (int j = 0; j <= slices; ++j) {
            z = -L + L*2/slices*j;
            if (pos = glm::vec3(x, y, z); vertexMap.find(pos) == vertexMap.end()) {
                vertexMap[pos] = index;
                nodesMass.push_back(mass);
                nodesPos.push_back(pos);
                indexs.push_back(index);
                index++;
            }
            else {
                indexs.push_back(vertexMap[pos]);
            }
        }
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

    model.nodesMass = nodesMass;
    model.nodesPos = nodesPos;
    model.faces = vector<unsigned int>(faces.begin(), faces.end());

    if (soft) {
        model.type = 0;
        model.tetra = tetra;

        if (tetra == true) {  // Using Tetrahedrons (uses tetgen)
            int surfaceType = 1; // 1 = box
            calculateTetras(model, compliance, centerCompliance, mass, surfaceType);
        }
        else {
            calculateSprings(model, compliance);
            calculateAngledSprings(model, angleStiffness);
        }

        model.volumeCompliance = volumeCompliance;
        model.volume = calculateVolume(model.nodesPos, model.faces);
        model.subStep = subStep;
    }
    else {
        model.type = 1;
        glm::vec3 center = glm::vec3(0, 0, 0);
        model.nodesMass.push_back(mass);
        model.nodesPos.push_back(center);
    }

    calculateSpringMap(model);

    computeVertexNormals(model.nodesNor, model.nodesPos, model.faces);

    createBB(model);
    createModelVBHTetra(model);

    printf("Number of points %d Number of Faces %d Number of springs %d \n", static_cast<int>(model.nodesPos.size()), static_cast<int>(model.faces.size()), static_cast<int>(model.springs.size()));
    return model;
}

void writeModel (Model &model, string saveDirectory, string modelName) {
    if (!fs::exists(saveDirectory)) {
        if (fs::create_directory(saveDirectory)) {
            cout << "Directory created: " << saveDirectory << '\n';
        } else {
            cerr << "Failed to create directory: " << saveDirectory << '\n';
            return;
        }
    }

    string path = saveDirectory + "/" + modelName + ".json";

    if (!fs::exists(path)) {
        ofstream file(path);
        if (file) {
            cout << "File created: " << path << '\n';
            file.close();
        } else {
            cerr << "Failed to create file: " << path << '\n';
            return;
        }
    }

    ofstream modelFile (path);
    if (modelFile.is_open())
    {
        modelFile << json(model).dump(4);
        cout << "File Writened: " << path << '\n';
    }
    else cerr << "Unable to open file";
    return;
}

void main() {
    int type;
    string directory;
    string name;

    cout << "Enter the model directory: ";
    cin >> directory;

    cout << "Enter the model name: ";
    cin >> name;

    cout << "Enter which type of object to create (0 = icosphere, 1 = globe sphere, 2 = box): ";
    cin >> type;

    Model m;

    if (type == 0) {
        bool soft, tetra;
        float radius, mass, bounciness, colorX, colorY, colorZ, subStep, volumeCompliance, compliance, centerCompliance, angleStiffness;
        int detail;

        cout << "Enter model soft: ";
        cin >> soft;

        cout << "Enter the radius: ";
        cin >> radius;

        cout << "Enter the detail: ";
        cin >> detail;

        cout << "Enter the mass: ";
        cin >> mass;

        cout << "Enter the bounciness: ";
        cin >> bounciness;

        cout << "Enter the red aspect of the color: ";
        cin >> colorX;

        cout << "Enter the blue aspect of the color: ";
        cin >> colorY;

        cout << "Enter the green aspect of the color: ";
        cin >> colorZ;

        glm::vec3 color = glm::vec3(colorX, colorY, colorZ);

        if (soft) {
            cout << "Enter the subStep: ";
            cin >> subStep;

            cout << "Enter the volumeCompliance: ";
            cin >> volumeCompliance;

            cout << "Enter the compliance: ";
            cin >> compliance;

            cout << "Enter the centerCompliance: ";
            cin >> centerCompliance;

            cout << "Enter the angleStiffness: ";
            cin >> angleStiffness;

            cout << "Enter the tetra usage: ";
            cin >> tetra;

            m = makeIcoSphere(radius, detail, subStep, mass, bounciness, volumeCompliance, compliance, centerCompliance, angleStiffness, color, tetra, soft);
            writeModel(m, directory, name);
        }
        else {
            subStep = 0;
            volumeCompliance = 0;
            compliance = 0;
            centerCompliance = 0;
            angleStiffness = 0;
            tetra = false;

            m = makeIcoSphere(radius, detail, subStep, mass, bounciness, volumeCompliance, compliance, centerCompliance, angleStiffness, color, tetra, soft);
            writeModel(m, directory, name);
        }
    }
    if (type == 1) {
        bool soft, tetra;
        float radius, mass, bounciness, colorX, colorY, colorZ, subStep, volumeCompliance, compliance, centerCompliance, angleStiffness;
        int slices, stacks;

        cout << "Enter model soft: ";
        cin >> soft;

        cout << "Enter the radius: ";
        cin >> radius;

        cout << "Enter the slices: ";
        cin >> slices;

        cout << "Enter the stacks: ";
        cin >> stacks;

        cout << "Enter the mass: ";
        cin >> mass;

        cout << "Enter the bounciness: ";
        cin >> bounciness;

        cout << "Enter the red aspect of the color: ";
        cin >> colorX;

        cout << "Enter the blue aspect of the color: ";
        cin >> colorY;

        cout << "Enter the green aspect of the color: ";
        cin >> colorZ;

        glm::vec3 color = glm::vec3(colorX, colorY, colorZ);

        if (soft) {
            cout << "Enter the subStep: ";
            cin >> subStep;

            cout << "Enter the volumeCompliance: ";
            cin >> volumeCompliance;

            cout << "Enter the compliance: ";
            cin >> compliance;

            cout << "Enter the centerCompliance: ";
            cin >> centerCompliance;

            cout << "Enter the angleStiffness: ";
            cin >> angleStiffness;

            cout << "Enter the tetra usage: ";
            cin >> tetra;

            m = makeSphere(radius, slices, stacks, subStep, mass, bounciness, volumeCompliance, compliance, centerCompliance, angleStiffness, color, tetra, soft);
            writeModel(m, directory, name);
        }
        else {
            subStep = 0;
            volumeCompliance = 0;
            compliance = 0;
            centerCompliance = 0;
            angleStiffness = 0;
            tetra = false;

            m = makeSphere(radius, slices, stacks, subStep, mass, bounciness, volumeCompliance, compliance, centerCompliance, angleStiffness, color, tetra, soft);
            writeModel(m, directory, name);
        }
    }
    if (type == 2) {
        bool soft, tetra;
        float H, V, L, mass, bounciness, colorX, colorY, colorZ, subStep, volumeCompliance, compliance, centerCompliance, angleStiffness;
        int slices, stacks;

        cout << "Enter model soft: ";
        cin >> soft;

        cout << "Enter the Half Horizontal X length: ";
        cin >> H;

        cout << "Enter the Half Vertical length: ";
        cin >> V;

        cout << "Enter the Half Horizontal Z length: ";
        cin >> L;

        cout << "Enter the slices: ";
        cin >> slices;

        cout << "Enter the stacks: ";
        cin >> stacks;

        cout << "Enter the mass: ";
        cin >> mass;

        cout << "Enter the bounciness: ";
        cin >> bounciness;

        cout << "Enter the red aspect of the color: ";
        cin >> colorX;

        cout << "Enter the blue aspect of the color: ";
        cin >> colorY;

        cout << "Enter the green aspect of the color: ";
        cin >> colorZ;

        glm::vec3 color = glm::vec3(colorX, colorY, colorZ);

        if (soft) {
            cout << "Enter the subStep: ";
            cin >> subStep;

            cout << "Enter the volumeCompliance: ";
            cin >> volumeCompliance;

            cout << "Enter the compliance: ";
            cin >> compliance;

            cout << "Enter the centerCompliance: ";
            cin >> centerCompliance;

            cout << "Enter the angleStiffness: ";
            cin >> angleStiffness;

            cout << "Enter the tetra usage: ";
            cin >> tetra;

            m = makeBox(slices, stacks, subStep, mass, bounciness, volumeCompliance, compliance, centerCompliance, angleStiffness, H, V, L, color, tetra, soft);
            writeModel(m, directory, name);
        }
        else {
            subStep = 0;
            volumeCompliance = 0;
            compliance = 0;
            centerCompliance = 0;
            angleStiffness = 0;
            tetra = false;

            m = makeBox(slices, stacks, subStep, mass, bounciness, volumeCompliance, compliance, centerCompliance, angleStiffness, H, V, L, color, tetra, soft);
            writeModel(m, directory, name);
        }
    }
}
