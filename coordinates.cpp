#include "coordinates.h"

Coordinates::Coordinates() : gap(0.5f), border(10.0f) 
{   
    createCoordinates(); 
    setup(); 
}


Coordinates::Coordinates(float gap, float border)
{
    this->gap = gap; 
    this->border = border; 
    createCoordinates(); 
    setup();
}

void Coordinates::createCoordinates() 
{
    points.push_back(glm::vec3(0, border, 0));
    //points.push_back(glm::vec3(0, -border, 0));
    points.push_back(glm::vec3(0, 0, 0));

    points.push_back(glm::vec3(border, 0, 0));
    points.push_back(glm::vec3(-border, 0, 0));
    points.push_back(glm::vec3(0, 0, border));
    points.push_back(glm::vec3(0, 0, -border));
   
    for (float line = 0; line < border; line += gap) {
        points.push_back(glm::vec3(border, 0, line));
        points.push_back(glm::vec3(-border, 0, line));
        points.push_back(glm::vec3(line, 0, border));
        points.push_back(glm::vec3(line, 0, -border));
        points.push_back(glm::vec3(border, 0, -line));
        points.push_back(glm::vec3(-border, 0, -line));
        points.push_back(glm::vec3(-line, 0, border));
        points.push_back(glm::vec3(-line, 0, -border));
    }


}

void Coordinates::setup()
{

    // create buffers/arrays
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);

    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, points.size()*sizeof(glm::vec3), &points[0], GL_STATIC_DRAW);

    // set the vertex attribute pointers
    // vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);

    // unbind VAO
    glBindVertexArray(0);
}


void Coordinates::clearMesh() 
{
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    setup();
}

void Coordinates::Draw(Shader shader) 
{
    // draw mesh
    glBindVertexArray(VAO);

    glDrawArrays(GL_LINES, 0, points.size());

    // unbind VAO
    glBindVertexArray(0);
}