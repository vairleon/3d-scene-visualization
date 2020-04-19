#ifndef MESH_H
#define MESH_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <iostream>
#include <cmath>
#include "shader.h"

#ifndef NAMESPACE_STD
#define NAMESPACE_STD
using namespace std;
#endif

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Normal;
    glm::vec3 Color;
    //glm::vec3 
};

struct Meshinfo {
    unsigned int id;
    int label; //
    int m_face; //triangle or poly
    float prob;
    //string info;
    Meshinfo() : id(0), label(0), prob(0.0f), m_face(3){};
};



class Mesh { //single mesh
public:
    /*  ��������  */
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    Meshinfo meshinfo;

    unsigned int VAO;
    bool hasNormals;

    /*  ����  */
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, Meshinfo meshinfo);
    void Draw(Shader shader);

    // --- utils --- //
    bool isFaceNormals();
    void facesGenerate();
    void pclNomalization(); // test
    

private:
    /*  ��Ⱦ����  */
    static unsigned int id; //
    unsigned int VBO, EBO;
    /*  ����  */
    void setupMesh();
    void normalsGeneration();
    void faceReconstruction();
};

void faceReconstruction(const vector<Vertex>& vetices, vector<unsigned int> indices); // generate faces from point clouds

#endif