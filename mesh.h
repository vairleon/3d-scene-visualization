#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <iostream>

#include "shader.h"

using namespace std;

struct Vertex {
    glm::vec3 Position;
    glm::vec3 Color;
};


class Mesh {
public:
    /*  ��������  */
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    unsigned int VAO;
    /*  ����  */
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices);
    void Draw(Shader shader);
private:
    /*  ��Ⱦ����  */
    unsigned int VBO, EBO;
    /*  ����  */
    void setupMesh();
};
