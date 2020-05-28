#ifndef COORDINATES_H
#define COORDINATES_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <vector>
#include <iostream>

#include "shader.h"

// -- draw Coordinate systems --

class Coordinates { 
public:
    /*  网格数据  */
    std::vector<glm::vec3> points;

    unsigned int VAO;

    /*  函数  */
    Coordinates(); 
    Coordinates(float gap, float border);

    
    void Draw(Shader shader);
    void clearMesh();


private:

    /* settings */
    float gap, border;

    /*  渲染数据  */
    unsigned int VBO, EBO;

    /*  函数  */
    void setup();
    void createCoordinates();
};


#endif