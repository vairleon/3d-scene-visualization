#ifndef MODEL_H
#define MODEL_H

#include <glad/glad.h> 

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

//#define STB_IMAGE_IMPLEMENTATION
//#include <stb_image.h>

#include "mesh.h"
#include "shader.h"
#include "mesh_io.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <regex>
#include <cstdlib>
#include <cstring>

enum Lable { backgound = -1, Mon, Tue, Wed, Thu, Fri, Sat };

class Model  //file - level
{
public:
    /*  函数   */
    Model(char* path) //暂时不用这个
    {
        loadModel(path);
    }
    Model(vector<string> path);
     
    void Draw(Shader shader);
    void Draw(Shader shader, unsigned int mesh_id);
    void meshNormalization(unsigned int mesh_id);
    void saveMesh(string path); //ply format
    void replaceMeshWithCADmodel(unsigned int mesh_id);
       
private:
    /*  模型数据  */
    vector<Mesh> meshes;
    //string directory;
    /*  函数   */
    void loadModel(string path);
    void loadMesh(string path);
    
    /*  ICP test */
    void ICP(unsigned int meshid_a, unsigned int meshid_b);
};

#endif