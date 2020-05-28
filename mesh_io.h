#ifndef MESH_IO_H
#define MESH_IO_H


#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <cstring>

#include "mesh.h"
#include "utils\utils.h"

#define MASK_PATH  "model\\scenes\\benchmark_result\\"


void plyParser(const vector<string>& plycode, vector<Mesh>& meshes); //  .ply file parser func
void objParser(const vector<string>& objcode, vector<Mesh>& meshes); //  .obj file parser func
void objParser(const vector<string>& objcode, Mesh& mesh); //  .obj file parser func
void plyWriter(string path, const Mesh mesh);		// .ply file writer from mesh
void objParserPointCloud(const vector<string>& objcode, vector<Vertex>& vertices); // read .obj file (vertices only) ps: can be combined to objParser() later

#endif