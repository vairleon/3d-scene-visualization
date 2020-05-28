#ifndef MESH_H
#define MESH_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h> 

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
    //unsigned int _;
    unsigned int id;
    int label; //    
    int m_face; //triangle or poly
    float prob;
    glm::mat4 trans;
    bool visible;
    //string info;

    void copy(const Meshinfo& other) {
        this->label = other.label;
        this->m_face = other.m_face;
        this->prob = other.prob;
        this->trans = other.trans;
        this->visible = other.visible;
        //info = other.info;
    }

    //Meshinfo& operator=(const Meshinfo& other) {
    //    id = other.id;
    //    label = other.label;
    //    m_face = other.m_face;
    //    prob = other.prob;
    //    trans = other.trans;
    //    visible = other.visible;
    //    //info = other.info;
    //    return *this;
    //}

    Meshinfo() : id(0), label(0), prob(0.0f), m_face(3), trans(1.0f), visible(true){};
};



class Mesh { //single mesh
public:
    /*  网格数据  */
    Meshinfo meshinfo;
    vector<Vertex> vertices;
    vector<unsigned int> indices;

    unsigned int VAO;


    /*  函数  */
    Mesh(vector<Vertex> vertices, vector<unsigned int> indices, Meshinfo meshinfo);
    Mesh();
    void Draw(Shader shader);

    // --- utils --- //
    bool isFaceNormals();
    void Nomalization(); // test
    void toPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void Mesh::destroyMesh();
    void Mesh::buildMesh();

private:
    /*  渲染数据  */
    static unsigned int id; //
    unsigned int VBO, EBO;
    bool hasNormals;
    /*  函数  */
    void setupMesh();
    void normalGeneration();
    void faceReconstruction();
};

void faceReconstruction(const vector<Vertex>& vetices, vector<unsigned int> indices); // generate faces from point clouds

#endif