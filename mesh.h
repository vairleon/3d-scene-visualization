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
    /*  网格数据  */
    vector<Vertex> vertices;
    vector<unsigned int> indices;
    unsigned int VAO;
    /*  函数  */
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices);
    void Draw(Shader shader);
private:
    /*  渲染数据  */
    unsigned int VBO, EBO;
    /*  函数  */
    void setupMesh();
};
