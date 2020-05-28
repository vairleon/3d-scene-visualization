#include "model.h"


// the interface for 3d-sis segmentation result
void sceneParser(vector<Mesh>& meshes, const string filename);	     //  scenes split through segmentation masks



// class functions
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

void Model::Draw(Shader shader, int seq) {
	meshes[seq].Draw(shader);
}

void Model::loadMesh(string path) {

	// 从文件路径中获取模型文件
	vector<string> codes; //读文档
	std::ifstream modelFile;
	// 保证ifstream对象可以抛出异常：
	//modelFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// 打开文件
		string code;
		modelFile.open(path, std::ifstream::in);
		while (!modelFile.eof()) {
			std::getline(modelFile, code);
			codes.push_back(code);
		}
		modelFile.close();
		// 转换数据流到string
		// plyCode = PlyStream.str();
	}
	catch (std::ifstream::failure e)
	{
		std::cout << "ERROR::MODEL_PLYFILE::FILE_NOT_SUCCESFULLY_READ" << std::endl;
	}

	
	// 利用plyCode创建Mesh对象序列
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


void Model::loadCADMesh(string path, Mesh& mesh) {

	// 从文件路径中获取模型文件
	vector<string> codes; //读文档
	std::ifstream modelFile;
	// 保证ifstream对象可以抛出异常：
	//modelFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// 打开文件
		string code;
		modelFile.open(path, std::ifstream::in);
		while (!modelFile.eof()) {
			std::getline(modelFile, code);
			codes.push_back(code);
		}
		modelFile.close();
		// 转换数据流到string
		// plyCode = PlyStream.str();
	}
	catch (std::ifstream::failure e)
	{
		std::cout << "Model.cpp:ERROR::MODEL_PLYFILE::FILE_NOT_SUCCESFULLY_READ" << std::endl;
	}

	objParser(codes, mesh);
}

void Model::saveMesh(string path) {
	for (vector<Mesh>::iterator it = meshes.begin(); it != meshes.end(); it++) {
		if ((*it).isFaceNormals()) { // transform .obj file into .ply file 
			continue;	// to be updated soon
		}
		(*it).Nomalization();
		plyWriter(path, *it);
		//std::cout << "save mesh " << (*it).meshinfo.id << endl;
	}
}

void Model::saveMesh(string path, unsigned int id) {
	for (int i = 0; i < meshes.size(); i++) {
		if (i == id) {
			if (meshes[i].isFaceNormals()) { // transform .obj file into .ply file 
				continue;	// to be updated soon
			}
			meshes[i].Nomalization();
			plyWriter(path, meshes[i]);
		}
		//std::cout << "save mesh " << (*it).meshinfo.id << endl;
	}
}

void Model::meshNormalization(unsigned int mesh_id) {
	for (unsigned int i = 0; i < meshes.size(); i++) {
		if (mesh_id == meshes[i].meshinfo.id)
			meshes[i].Nomalization();
	}

}
map<unsigned int, unsigned int> Model::getReplacingMap() {
	return this->replacingMap;
}

int Model::getMeshNum() {
	return meshes.size();
}
unsigned int Model::getMeshId(int seq) {
	return meshes[seq].meshinfo.id;
}
int Model::getMeshLabel(int seq) {
	return meshes[seq].meshinfo.label;
}
int Model::getMeshPointsNum(int seq) {
	if (meshes[seq].isFaceNormals())
		return -1;
	return meshes[seq].vertices.size();
}
glm::mat4 Model::getMeshTransformation(int seq) {
	return meshes[seq].meshinfo.trans;
}

bool Model::getIfVisible(int seq) {
	return meshes[seq].meshinfo.visible;
}
Meshinfo Model::getMeshInfo(int seq) {
	return meshes[seq].meshinfo;
}

void Model::centerMeshes() {
	
	int pointnum = 0; 
	for (int i = 0; i < meshes.size(); i++) {
		pointnum += meshes[i].vertices.size();
		meshes[i].destroyMesh();
	}

	glm::vec3 centroid = glm::vec3(0.0f);
	for (int i = 0; i < meshes.size(); i++) {
		for (int j = 0; j < meshes[i].vertices.size(); j++) {
			centroid += meshes[i].vertices[j].Position;
		}
	}
	centroid /= (float)pointnum;

	for (int i = 0; i < meshes.size(); i++) {
		for (int j = 0; j < meshes[i].vertices.size(); j++) {
			meshes[i].vertices[j].Position -= centroid;
		}
		meshes[i].buildMesh();
	}
}

void Model::replaceMeshWithCADmodel(unsigned int mesh_id) {

	std::cout << "start replacement ... waiting..." << std::endl;

	for (unsigned int i = 0; i < meshes.size(); i++) {
		if (mesh_id != meshes[i].meshinfo.id)
			continue;
		
		if (meshes[i].isFaceNormals() || !meshes[i].meshinfo.visible)
			return ;

		PointCloudT::Ptr cloud_src(new PointCloudT);
		PointCloudT::Ptr cloud_tgt(new PointCloudT);
		meshes[i].toPointCloud(cloud_tgt);

		// search cad database for similar target model, and load it's pointcloud
		string path = query(cloud_tgt, cloud_src);

		// load cad model 
		Mesh mesh;
		loadCADMesh(path, mesh);
		mesh.meshinfo.copy(meshes[i].meshinfo);

		// alginment
		Registration reg(cloud_src, cloud_tgt);	
		reg.regMethodBySacIaAndIcp();
		mesh.meshinfo.trans = EigenMatrix4fToGlmMat4(reg.getFinalTransformation());
		
		// replace
		meshes[i].meshinfo.visible = false;
		meshes.push_back(mesh);
		
		// update replacing map
		addReplacingMap(meshes[i].meshinfo.id, meshes.end()->meshinfo.id);

		break;
	}
}

void Model::replaceMeshWithCADmodel(int seq) {

	std::cout << "start replacement ... waiting..." << std::endl;
	if (meshes[seq].isFaceNormals() || !meshes[seq].meshinfo.visible)
		return;

	PointCloudT::Ptr cloud_src(new PointCloudT);
	PointCloudT::Ptr cloud_tgt(new PointCloudT);
	meshes[seq].toPointCloud(cloud_tgt);

	// search cad database for similar target model
	string path = query(cloud_tgt, cloud_src);

	// load cad model 
	Mesh mesh;
	loadCADMesh(path, mesh);
	mesh.meshinfo.copy(meshes[seq].meshinfo);

	// alginment
	Registration reg(cloud_src, cloud_tgt);		
	reg.regMethodBySacIaAndIcp();
	mesh.meshinfo.trans = EigenMatrix4fToGlmMat4(reg.getFinalTransformation());

	// replace
	meshes[seq].meshinfo.visible = false;
	meshes.push_back(mesh);

	// update replacing map
	addReplacingMap(meshes[seq].meshinfo.id, meshes.end()->meshinfo.id);
}


void Model::replaceMeshWithCADmodel(unsigned int mesh_id, int method) {

	std::cout << "start replacement with method == 0 ... waiting..." << std::endl;

	for (unsigned int i = 0; i < meshes.size(); i++) {
		if (mesh_id != meshes[i].meshinfo.id)
			continue;

		if (meshes[i].isFaceNormals() || !meshes[i].meshinfo.visible)
			return;

		PointCloudT::Ptr cloud_src(new PointCloudT);
		PointCloudT::Ptr cloud_tgt(new PointCloudT);
		meshes[i].toPointCloud(cloud_tgt);

		// search cad database for similar target model, and load it's pointcloud
		string path = query(cloud_tgt, cloud_src);

		// load cad model 
		Mesh mesh;
		loadCADMesh(path, mesh);
		mesh.meshinfo.copy(meshes[i].meshinfo);

		// alginment
		Registration reg(cloud_src, cloud_tgt);

		switch (method)
		{
		case 0:
			reg.regMethodBySacIaAndIcpWithAlignedFloor(); break;
		case 1:
			reg.regMethodBySacIaAndIcp(); break;
		/*case 2:
			reg.regMethodByBruteAlignAndIcp*/
		default:
			std::cout << "set the wrong param 'method'." << std::endl;
			return ;
		}
		
		mesh.meshinfo.trans = EigenMatrix4fToGlmMat4(reg.getFinalTransformation());

		// replace
		meshes[i].meshinfo.visible = false;
		meshes.push_back(mesh);

		// update replacing map
		addReplacingMap(meshes[i].meshinfo.id, meshes.end()->meshinfo.id);

		break;
	}

	


	
}

void Model::modelTransformation(const Eigen::Matrix4f matrix) {
	
	glm::mat4 trans = EigenMatrix4fToGlmMat4(matrix);

	for (unsigned int i = 0; i < meshes.size(); i++) {

		// destroy VAO VBO binding
		meshes[i].destroyMesh();
		
		// excute transformations, both for positions & normals
		for (int index = 0; index < meshes[i].vertices.size(); index++) {
			glm::vec4 temp_result = trans * glm::vec4(meshes[i].vertices[index].Position, 1.0);
			meshes[i].vertices[index].Position = glm::vec3(temp_result[0], temp_result[1], temp_result[2]);	
			meshes[i].vertices[index].Normal = glm::mat3(transpose(inverse(trans))) * meshes[i].vertices[index].Normal;

		}

		// build VAO VBO binding
		meshes[i].buildMesh();
	}
}

void Model::modelTransformation(const glm::mat4 trans) {

	for (unsigned int i = 0; i < meshes.size(); i++) {

		// destroy VAO VBO binding
		meshes[i].destroyMesh();

		// excute transformations, both for positions & normals
		for (int index = 0; index < meshes[i].vertices.size(); i++) {

			meshes[i].vertices[index].Position = glm::vec3(trans * glm::vec4(meshes[i].vertices[index].Position, 1.0));
			meshes[i].vertices[index].Normal = glm::mat3(transpose(inverse(trans))) * meshes[i].vertices[index].Normal;
		}

		// build VAO VBO binding
		meshes[i].buildMesh();
	}
}

// floor  < -- > label 0 / 1
void Model::floorAlignment() {
	
	Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

	for (int i = 0; i < meshes.size(); i++) {
		if (meshes[i].meshinfo.label != 0)
			continue;

		PointCloudT::Ptr cloud_tgt(new PointCloudT);
		meshes[i].toPointCloud(cloud_tgt);

		// load pointcloud
		FitPlane fitplane(cloud_tgt);
		
		// fit plane
		fitplane.ransacFitPlane();
		// fitplane.planeAlignment_rough(Eigen::Vector3d(0.0, 1.0, 0.0));
		fitplane.planeAlignment();
		// get transformationMatrix;
		trans = fitplane.getTransformationMatrix();

		// refine plane Meshes
		// ...

		
		// generate smooth plane from fitplane.Params
		Eigen::VectorXf planecoeff = fitplane.getCoeff();
		Mesh planemesh;
		planemesh.meshinfo.copy(meshes[i].meshinfo);
		planemesh.meshinfo.visible = false;
		float _z = 0;
		float _x[] = { -5.f, -5.f, 5.f, 5.f };
		float _y[] = { -5.f, 5.f, -5.f, 5.f };
		for (int i = 0; i < 4; i++) {
			_z = -(planecoeff[0] * _x[i] + planecoeff[1] * _y[i] + planecoeff[3]) / planecoeff[2];
			glm::vec3 position = glm::vec3(_x[i], _y[i], _z);
			glm::vec3 normal = glm::vec3(planecoeff[0], planecoeff[1], planecoeff[2]);
			glm::vec3 color = glm::vec3(0.0509f, 0.643f, 0.941f);
			planemesh.vertices.push_back({ position, normal, color });
		}

		planemesh.indices.push_back(0);
		planemesh.indices.push_back(1);
		planemesh.indices.push_back(2);
		planemesh.indices.push_back(1);
		planemesh.indices.push_back(2);
		planemesh.indices.push_back(3);

		// -------------------------------------------
		meshes.push_back(planemesh);
		break;
	}

	modelTransformation(trans);

}

void Model::addReplacingMap(const unsigned int ori_mesh_id, const unsigned int cad_mesh_id) 
{
	replacingMap.insert(std::pair<unsigned int, unsigned int>(ori_mesh_id, cad_mesh_id));
}

bool Model::delReplacingMap(const unsigned int ori_mesh_id)
{	
	/*
	map<unsigned int, unsigned int>::iterator iter = replacingMap.find(ori_mesh_id);
	if (iter == replacingMap.end()) {
	}*/
	return replacingMap.erase(ori_mesh_id);
}

void Model::discardUselessMeshes() {
	vector<Mesh>::iterator it = meshes.begin();
	
	while (it != meshes.end()) {
		if (!it->meshinfo.visible) {
			if (!delReplacingMap(it->meshinfo.id)) {
				it ++;
				continue;
			}
			it->destroyMesh();
			meshes.erase(it);
		}
		else {
			it++;
		}
	}
}

void Model::delMesh(unsigned int mesh_id) {


	for (vector<Mesh>::iterator it = meshes.begin(); it != meshes.end();) {
		if (mesh_id == it->meshinfo.id) {
			it->destroyMesh();
			meshes.erase(it);
			return;
		}
		else 
			it++;
	}
}

void Model::setVisible(unsigned int mesh_id) {
	for (int i = 0; i < meshes.size(); i++) {
		if (mesh_id != meshes[i].meshinfo.id)
			continue;

	
		meshes[i].meshinfo.visible = !meshes[i].meshinfo.visible;
		if(meshes[i].meshinfo.visible)
			meshes[i].buildMesh();
		else
			meshes[i].destroyMesh();

		break;
	}


}
void Model::autoRefineScenes() {
	// center pointcloud

	// floor alignment

	// fill hole of plane 
	
	// mesh replacement



	// texture calculation

	//
}




/* 可以做并行加速 */
void sceneParser(vector<Mesh>& meshes, const string filename) {

	std::cout << "start sceneParser..." << std::endl;
	clock_t time_start = clock();

	vector<string> tokens;
	split(filename, tokens, ".");
	string file_pre = tokens[0]; //去掉后缀
	vector<string> lines; //读文档
	std::ifstream infoFile;
	// 保证ifstream对象可以抛出异常：
	infoFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// 打开文件
		string line;
		infoFile.open(MASK_PATH + file_pre + ".txt", std::ifstream::in);
		while (!infoFile.eof()) {
			std::getline(infoFile, line);
			lines.push_back(line);
		}
		infoFile.close();
		// 转换数据流到string
		// plyCode = PlyStream.str();
	}
	catch (std::ifstream::failure e)
	{
		std::cout << "Model.cpp: ERROR::MASKINFO_FILE::FILE_NOT_SUCCESFULLY_READ" << std::endl;
	}


	vector<Mesh>::iterator it_scene = meshes.begin();
	vector<int> vertex_mask_label((*it_scene).vertices.size()); //记录mask类别
	vector<int> mask_index((*it_scene).vertices.size()); //记录vertex新下标

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
		// -- 再定义一个流
		std::ifstream infoFile;
		//infoFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
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
			std::cout << "Model.cpp:ERROR::MASKINFO_FILE::FILE_NOT_SUCCESFULLY_READ" << std::endl;
		}

		/* vertices  */
		for (unsigned int i = 0; i < mask.size(); i++) {
			if (!mask[i]) continue;
			vertices[l].push_back((*it_scene).vertices[i]);
			vertex_mask_label[i] = l; //按mask分类 ， 0为background
			mask_index[i] = vertices[l].size() - 1; //记录 vertex 原下标
		}

		// faceReconstruction(vertices, indices); // reconstruct faces
	}

	// background
	meshinfo[0].label = 0; // bkground
	meshinfo[0].prob = 1.0f;

	for (unsigned int i = 0; i < vertex_mask_label.size(); i++) {
		if (vertex_mask_label[i]) continue; // if 0 - 为背景标记
		vertices[0].push_back((*it_scene).vertices[i]);
		mask_index[i] = vertices[0].size() - 1; //记录 vertex 原下标
	}

	///* indices - got faces */
	vector<unsigned int> temp_indices = (*it_scene).indices;
	for (unsigned int i = 0; i < temp_indices.size(); i += 3) {
		unsigned int _index[3];
		int _flag[3];
		for (unsigned int t = 0; t < 3; t++) {
			_index[t] = temp_indices[i + t]; //原顶点下标
			_flag[t] = vertex_mask_label[_index[t]];
			_index[t] = mask_index[_index[t]]; //转换为新mesh的顶点索引
		}
		if (_flag[0] == _flag[1] && _flag[1] == _flag[2]) { //转换
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

	//...
	clock_t time_end = clock();
	cout << "sceneParser finished within " << 1000 * (time_end - time_start) / (double)CLOCKS_PER_SEC << "ms" << endl;

}