#ifndef MODEL_H
#define MODEL_H
#include <glad/glad.h> 

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

//#define STB_IMAGE_IMPLEMENTATION
//#include <stb_image.h>

#include "mesh.h"
#include "shader.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <regex>

class Model  //.ply file loader
{
public:
    /*  函数   */
    Model(char* path) //暂时不用这个
    {
        loadModel(path);
    }
    Model(vector<string> path);
     
    void Draw(Shader shader);
private:
    /*  模型数据  */
    vector<Mesh> meshes;
    string directory;
    /*  函数   */
    void loadModel(string path);
    Mesh loadMesh(string path);

};

#endif