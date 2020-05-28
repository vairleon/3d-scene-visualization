#include "mesh.h"

unsigned int Mesh::id = 0;

Mesh::Mesh(vector<Vertex> vertices, vector<unsigned int> indices, Meshinfo meshinfo) : hasNormals(true) {

    this->vertices = vertices;
    this->indices = indices;
    this->meshinfo = meshinfo;
    // setup mesh obj , bind mesh with VAO VBO EBO
    const int temp = id;
    this->meshinfo.id = temp;
    id++;
    setupMesh();
}

Mesh::Mesh() { 
    const int temp = id;
    this->meshinfo.id = temp;
    id++;
}

// render the mesh
void Mesh::Draw(Shader shader)
{
    // draw mesh
    glBindVertexArray(VAO);
    if (!isFaceNormals()) {
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
    }
    else {
        glDrawArrays(GL_TRIANGLES, 0, vertices.size());
    }
    // unbind VAO
    glBindVertexArray(0);
}

void Mesh::setupMesh()
{
  
    // create buffers/arrays
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);

    if (!isFaceNormals()) {
        glGenBuffers(1, &EBO);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), &indices[0], GL_STATIC_DRAW);
    }

    // set the vertex attribute pointers
    // vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    // vertex Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));
    // vertex Color
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Color));

    // unbind VAO
    glBindVertexArray(0);
}



#define MINFLOAT -1000000.0f
#define MAXFLOAT 1000000.0f
void Mesh::Nomalization() {  // nomalizing pclouds to (-1, 1)

    float max_x = MINFLOAT, min_x = MAXFLOAT;
    float max_y = MINFLOAT, min_y = MAXFLOAT;
    float max_z = MINFLOAT, min_z = MAXFLOAT;

    glm::vec3 pcenter = glm::vec3(0.0f, 0.0f, 0.0f);
    for (vector<Vertex>::iterator it = vertices.begin(); it != vertices.end(); it++) {
        glm::vec3 pos = (*it).Position;
        if (pos.x > max_x) {
            max_x = pos.x;
        }
        if (pos.x < min_x) {
            min_x = pos.x;
        }
        if (pos.y > max_y) {
            max_y = pos.y;
        }
        if (pos.y < min_y) {
            min_y = pos.y;
        }
        if (pos.z > max_z) {
            max_z = pos.z;
        }
        if (pos.z < min_z) {
            min_z = pos.z;
        }
        pcenter += pos;
    }

    pcenter /= vertices.size();

#define max(a,b) ((a) > (b) ? (a) : (b))

    for (vector<Vertex>::iterator it = vertices.begin(); it != vertices.end(); it++) {
        (*it).Position -= pcenter;
        (*it).Position.x /= max(abs(max_x - pcenter.x), abs(min_x - pcenter.x));
        (*it).Position.y /= max(abs(max_y - pcenter.y), abs(min_y - pcenter.y));
        (*it).Position.z /= max(abs(max_z - pcenter.z), abs(min_z - pcenter.z));
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    setupMesh();
}

void Mesh::destroyMesh() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    
}

void Mesh::buildMesh() {
    setupMesh();
}

bool Mesh::isFaceNormals() {
    return indices.size() > 0 ? false: true;
}

void Mesh::toPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
    if (isFaceNormals()) {
        cout << "Error: cannot turn this mesh into PointCloud types" << endl;
        return ;
    }
    for (unsigned int i = 0; i < vertices.size(); i++) {
        (*cloud).push_back(pcl::PointXYZ(vertices[i].Position.x, vertices[i].Position.y, vertices[i].Position.z));
    }
}
