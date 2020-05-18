#ifndef MODEL_H
#define MODEL_H


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

//#define STB_IMAGE_IMPLEMENTATION
//#include <stb_image.h>

#include "mesh.h"
#include "shader.h"
#include "mesh_io.h"
#include "caddb.h"
#include "utils/utils.h"
#include "lib/registration.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <regex>
#include <cstdlib>
#include <cstring>
#include <ctime>

enum Lable { backgound = -1, Mon, Tue, Wed, Thu, Fri, Sat };

class Model  //file - level
{
public:
    /*  funcs   */
    Model(const Model & model) {
        meshes = model.meshes;
        for (int i = 0; i < meshes.size(); i++) {
            meshes[i].clearMesh();
        }
    };
    Model() {};
    Model(char* path) 
    {
        loadModel(path);
    }
    Model(vector<string> path);
    

    void Draw(Shader shader);
    void Draw(Shader shader, unsigned int mesh_id);
    void Draw(Shader shader, int seq);
    void saveMesh(string path); //ply format
    void saveMesh(string path, unsigned int mesh_id);

    // get mesh infomation
    int getMeshNum();
    int getMeshId(int seq);
    int getMeshLabel(int seq);
    int getMeshPointsNum(int seq);
    glm::mat4 getMeshTransformation(int seq);
    bool getIfVisible(int seq);
    Meshinfo getMeshInfo(int seq);

    // refine scenes
    
    void autoRefineScenes();  // waiting for update .........

    void replaceMeshWithCADmodel(unsigned int mesh_id);
    void replaceMeshWithCADmodel(int seq);
    void floorAlignment(); // transform entire model(floor) to surface-xy.
    void transformMeshes();
    void centerMeshes();

    // test utils
    void meshNormalization(unsigned int mesh_id);
private:
    /*  模型数据  */
    vector<Mesh> meshes;
    //string directory;

    /*  func   */
    void loadModel(string path);
    void loadMesh(string path);
    void loadCADMesh(string path, Mesh& mesh);
    /*  ICP test */
    //void ICP(unsigned int meshid_a, unsigned int meshid_b);

    /*  model transformation */
    void modelTransformation(const Eigen::Matrix4f matrix);
   
};



#endif