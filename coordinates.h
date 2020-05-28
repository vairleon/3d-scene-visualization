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
    /*  ��������  */
    std::vector<glm::vec3> points;

    unsigned int VAO;

    /*  ����  */
    Coordinates(); 
    Coordinates(float gap, float border);

    
    void Draw(Shader shader);
    void clearMesh();


private:

    /* settings */
    float gap, border;

    /*  ��Ⱦ����  */
    unsigned int VBO, EBO;

    /*  ����  */
    void setup();
    void createCoordinates();
};


#endif