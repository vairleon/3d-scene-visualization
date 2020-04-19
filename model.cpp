#include "model.h"



void sceneParser(vector<Mesh>& meshes, const string filename);	     //  scenes split through segmentation masks

Model::Model(vector<string> path) {
	
	for (unsigned int i = 0; i < path.size(); i++) {
		loadMesh(path[i]);
	}
}

void Model::Draw(Shader shader) {
    for (unsigned int i = 0; i < meshes.size(); i++)
        meshes[i].Draw(shader);
}

void Model::Draw(Shader shader, unsigned int mesh_id) {
	for (unsigned int i = 0; i < meshes.size(); i++) {
		if(mesh_id == meshes[i].meshinfo.id)
			meshes[i].Draw(shader);
	}		
}


void Model::loadModel(string path) {
    return ;
}

void Model::loadMesh(string path) {

	// ���ļ�·���л�ȡģ���ļ�
	vector<string> codes; //���ĵ�
	vector<unsigned int> indices;
	vector<Vertex> vertexs;
	std::ifstream modelFile;
	// ��֤ifstream��������׳��쳣��
	modelFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// ���ļ�
		string code;
		modelFile.open(path, std::ifstream::in);
		while (!modelFile.eof()) {
			std::getline(modelFile, code);
			codes.push_back(code);
		}
		modelFile.close();
		// ת����������string
		// plyCode = PlyStream.str();
	}
	catch (std::ifstream::failure e)
	{
		std::cout << "ERROR::MODEL_PLYFILE::FILE_NOT_SUCCESFULLY_READ" << std::endl;
	}

	
	// ����plyCode����Mesh��������
	vector<Mesh> vmesh;
	if (path.find(".obj") != string::npos) {
		objParser(codes, vmesh);
	}
	else if(path.find(".ply") != string::npos) {
		plyParser(codes, vmesh);

		vector<string> tokens;
		split(path, tokens, "\\");
		string filename = tokens[tokens.size() - 1];
		if (filename.find("scene") != string::npos) {
			sceneParser(vmesh, filename);
		}		
	}

	for (unsigned int i = 0; i < vmesh.size(); i++)
		meshes.push_back(vmesh[i]);
}


void Model::saveMesh(string path) {
	for (vector<Mesh>::iterator it = meshes.begin(); it != meshes.end(); it++) {
		if ((*it).isFaceNormals()) { // transform .obj file into .ply file 
			continue;	// to be updated soon
		}
		plyWriter(path, *it);
		//std::cout << "save mesh " << (*it).meshinfo.id << endl;
	}
}

void Model::meshNormalization(unsigned int mesh_id) {
	for (unsigned int i = 0; i < meshes.size(); i++) {
		if (mesh_id == meshes[i].meshinfo.id)
			meshes[i].pclNomalization();
	}

}


/* ���������м��� */
void sceneParser(vector<Mesh>& meshes, const string filename) {

	std::cout << "start sceneParser..." << std::endl;

	vector<string> tokens;
	split(filename, tokens, ".");
	string file_pre = tokens[0]; //ȥ����׺
	vector<string> lines; //���ĵ�
	std::ifstream infoFile;
	// ��֤ifstream��������׳��쳣��
	infoFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// ���ļ�
		string line;
		infoFile.open(MASK_PATH + file_pre + ".txt", std::ifstream::in);
		while (!infoFile.eof()) {
			std::getline(infoFile, line);
			lines.push_back(line);
		}
		infoFile.close();
		// ת����������string
		// plyCode = PlyStream.str();
	}
	catch (std::ifstream::failure e)
	{
		std::cout << "ERROR::MASKINFO_FILE::FILE_NOT_SUCCESFULLY_READ" << std::endl;
	}


	vector<Mesh>::iterator it_scene = meshes.begin();
	vector<int> vertex_mask_label((*it_scene).vertices.size()); //��¼mask���
	vector<int> mask_index((*it_scene).vertices.size()); //��¼vertex���±�

	vector<vector<Vertex>> vertices(lines.size() + 1);
	vector<vector<unsigned int>> indices(lines.size() + 1);
	vector<Meshinfo> meshinfo(lines.size() + 1);

	for (unsigned int l = 1; l < lines.size() + 1; l++) {
		/* initialization */

		vector<string> tokens;
		/* got target mask file name */
		split(lines[l - 1], tokens);
		replace(tokens[0].begin(), tokens[0].end(), '/', '\\');
		string filepath = MASK_PATH + tokens[0]; // per-mask

		/* Meshinfo */
		meshinfo[l].label = stoi(tokens[1].c_str());
		meshinfo[l].prob = stof(tokens[2].c_str());
		// meshinfo.info = ...

		/* read target file*/
		vector<int> mask;
		// -- �ٶ���һ����
		std::ifstream infoFile;
		infoFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
		try
		{
			string line;
			infoFile.open(filepath, std::ifstream::in);
			while (!infoFile.eof()) {
				std::getline(infoFile, line);
				std::istringstream iss(line);
				int num;
				iss >> num;
				mask.push_back(num);
			}
			infoFile.close();
		}
		catch (std::ifstream::failure e)
		{
			std::cout << "ERROR::MASKINFO_FILE::FILE_NOT_SUCCESFULLY_READ" << std::endl;
		}

		/* vertices  */
		for (unsigned int i = 0; i < mask.size(); i++) {
			if (!mask[i]) continue;
			vertices[l].push_back((*it_scene).vertices[i]);
			vertex_mask_label[i] = l; //��mask���� �� 0Ϊbackground
			mask_index[i] = vertices[l].size() - 1; //��¼ vertex ԭ�±�
		}

		// faceReconstruction(vertices, indices); // reconstruct faces
	}

	// background
	meshinfo[0].label = 0; // bkground
	meshinfo[0].prob = 1.0f;

	for (unsigned int i = 0; i < vertex_mask_label.size(); i++) {
		if (vertex_mask_label[i]) continue; // if 0 - Ϊ�������
		vertices[0].push_back((*it_scene).vertices[i]);
		mask_index[i] = vertices[0].size() - 1; //��¼ vertex ԭ�±�
	}

	///* indices - got faces */
	vector<unsigned int> temp_indices = (*it_scene).indices;
	for (unsigned int i = 0; i < temp_indices.size(); i += 3) {
		unsigned int _index[3];
		int _flag[3];
		for (unsigned int t = 0; t < 3; t++) {
			_index[t] = temp_indices[i + t]; //ԭ�����±�
			_flag[t] = vertex_mask_label[_index[t]];
			_index[t] = mask_index[_index[t]]; //ת��Ϊ��mesh�Ķ�������
		}
		if (_flag[0] == _flag[1] && _flag[1] == _flag[2]) { //ת��
			indices[_flag[0]].push_back(_index[0]);
			indices[_flag[0]].push_back(_index[1]);
			indices[_flag[0]].push_back(_index[2]);
		}
	}


	/* pushback mesh */
	meshes.clear();
	for (unsigned int i = 0; i < lines.size() + 1; i++) {
		Mesh mesh = Mesh(vertices[i], indices[i], meshinfo[i]);
		meshes.push_back(mesh);
	}

}


void Model::replaceMeshWithCADmodel(unsigned int mesh_id) {


}
