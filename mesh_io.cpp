#include "mesh_io.h"


void plyWriter(string path, const Mesh mesh) {

	std::ofstream modelFile;
	// 保证ifstream对象可以抛出异常：
	modelFile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	try
	{
		// 打开文件
		string filename;
		string str_id = to_string(mesh.meshinfo.id);
		while (str_id.size() < 4) {
			str_id = '0' + str_id;
		}
		filename = path + str_id + "_" + to_string(mesh.meshinfo.label) + ".ply";
		modelFile.open(filename, std::ofstream::out | std::ofstream::trunc);

		modelFile << "ply" << std::endl;
		modelFile << "format ascii 1.0" << std::endl;
		modelFile << "comment VCGLIB generated" << std::endl;
		modelFile << "element vertex " << mesh.vertices.size() << std::endl;
		modelFile << "property float x" << std::endl;
		modelFile << "property float y" << std::endl;
		modelFile << "property float z" << std::endl;
		modelFile << "property float nx" << std::endl;
		modelFile << "property float ny" << std::endl;
		modelFile << "property float nz" << std::endl;
		modelFile << "property uchar red" << std::endl;
		modelFile << "property uchar green" << std::endl;
		modelFile << "property uchar blue" << std::endl;
		modelFile << "element face " << (unsigned)mesh.indices.size() / 3 << std::endl;
		modelFile << "property list uchar int vertex_indices" << std::endl;
		modelFile << "end_header" << std::endl;

#define meshPosition mesh.vertices[i].Position 
#define meshNormal mesh.vertices[i].Normal
#define meshColor mesh.vertices[i].Color

		for (unsigned int i = 0; i < mesh.vertices.size(); i++) {
			modelFile << meshPosition.x << " " << meshPosition.y << " " << meshPosition.z << " ";
			modelFile << meshNormal.x << " " << meshNormal.y << " " << meshNormal.z << " ";
			modelFile << (unsigned int)(meshColor.x * 255) << " " << (unsigned int)(meshColor.y * 255) << " " << (unsigned int)(meshColor.z * 255) << endl;
		}

		for (unsigned int i = 0; i < mesh.indices.size(); i += 3) {
			modelFile << 3 << " " << mesh.indices[i] << " " << mesh.indices[i + 1] << " " << mesh.indices[i + 2] << endl;
		}

		modelFile.close();
		// 转换数据流到string
		// plyCode = PlyStream.str();
	}
	catch (std::ofstream::failure e)
	{
		std::cout << "ERROR::PLY_OUTFILE::FILE_NOT_SUCCESFULLY_WRITE" << std::endl;
	}
}

// objParser
void objParser(const vector<string>& objcode, vector<Mesh>& meshes) {

	vector<Vertex> vertices;
	vector<glm::vec3> normals;
	vector<unsigned int>  v_indices;
	vector<unsigned int>  vt_indices;
	vector<unsigned int>  vn_indices;

	bool isFaceNormal = false;
	unsigned int codeindex = 0;

	while (codeindex < objcode.size()) {
		std::istringstream iss(objcode[codeindex]);
		string token;
		iss >> token;
		if (token == "v") {
			vector<float> vnum;
			float num = 0;
			while (iss >> num) { vnum.push_back(num); }
			if (vnum.size() < 4) {
				glm::vec3 vertex = glm::vec3(vnum[0], vnum[1], vnum[2]);
				glm::vec3 color = glm::vec3(1.0f, 0.5f, 0.31f);
				glm::vec3 normal = glm::vec3(0.0f, 0.0f, 0.0f);
				vertices.push_back({ vertex, normal, color });
			}
			else {
				glm::vec3 position = glm::vec3(vnum[0], vnum[1], vnum[2]);
				glm::vec3 color = glm::vec3(vnum[3], vnum[4], vnum[5]);
				glm::vec3 normal = glm::vec3(0.0f, 0.0f, 0.0f);
				vertices.push_back({ position, normal, color });
			}
		}
		else if (token == "vn") {
			vector<float> vnum;
			float num = 0;
			while (iss >> num) { vnum.push_back(num); }
			glm::vec3 normal = glm::vec3(vnum[0], vnum[1], vnum[2]);
			normals.push_back(normal);
		}
		else if (token == "f") {
			vector<string> tokens;
			split(objcode[codeindex], tokens);

			vector<unsigned int> temp_v_index;
			//vector<unsigned int> temp_vt_index;
			vector<unsigned int> temp_vn_index;
			for (int i = 1; i < tokens.size(); i++) {
				vector<string> str_index;
				split(tokens[i], str_index, "/");
				temp_v_index.push_back((unsigned int)atoi(str_index[0].c_str()) - 1);
				if (str_index.size() > 1) {
					temp_vn_index.push_back((unsigned int)atoi(str_index[str_index.size() - 1].c_str()) - 1);
				}
			}
			if (temp_v_index.size() == 4) {
				v_indices.push_back(temp_v_index[0]);
				v_indices.push_back(temp_v_index[1]);
				v_indices.push_back(temp_v_index[2]);
				v_indices.push_back(temp_v_index[1]);
				v_indices.push_back(temp_v_index[2]);
				v_indices.push_back(temp_v_index[3]);

				if (temp_vn_index.size()) {
					vn_indices.push_back(temp_vn_index[0]);
					vn_indices.push_back(temp_vn_index[1]);
					vn_indices.push_back(temp_vn_index[2]);
					vn_indices.push_back(temp_vn_index[1]);
					vn_indices.push_back(temp_vn_index[2]);
					vn_indices.push_back(temp_vn_index[3]);
				}
			}
			else if (temp_v_index.size() == 3) {
				v_indices.push_back(temp_v_index[0]);
				v_indices.push_back(temp_v_index[1]);
				v_indices.push_back(temp_v_index[2]);
				if (temp_vn_index.size()) {
					vn_indices.push_back(temp_vn_index[0]);
					vn_indices.push_back(temp_vn_index[1]);
					vn_indices.push_back(temp_vn_index[2]);
				}
				else {
					int a = 0;
				}
			}
			else {
				throw "Warning : Can't read this format of faces !";
			}
		}
		//else
		codeindex++;
	}

	if (normals.size() != vertices.size()) {
		isFaceNormal = true;
	}

	
	if (isFaceNormal) {
		vector<Vertex> vertices_f(v_indices.size());
		for (unsigned int i = 0; i < v_indices.size(); i++) {
			vertices_f[i].Position = vertices[v_indices[i]].Position;
			vertices_f[i].Normal = normals[vn_indices[i]];
			vertices_f[i].Color = vertices[v_indices[i]].Color;
		}

		Meshinfo meshinfo;
		v_indices.clear();
		meshes.push_back(Mesh(vertices_f, v_indices, meshinfo));
	}
	else {
		for (unsigned int i = 0; i < vertices.size(); i++) {
			vertices[i].Normal = normals[i];
		}
		Meshinfo meshinfo;
		meshes.push_back(Mesh(vertices, v_indices, meshinfo));
	}

	

	return;
}


// plyParser 只解析固定格式的plyfile 
void plyParser(const vector<string>& plycode, vector<Mesh>& meshes) {
	vector<Vertex> vertices;
	vector<unsigned int> indices;
	unsigned int codeindex = 0;
	unsigned int vertexnum = 0;
	unsigned int facenum = 0;
	while (plycode[codeindex++].find("ply") != string::npos) { if (codeindex >= plycode.size()) { return; } };
	while (plycode[codeindex].find("end_header") == string::npos) {
		if (plycode[codeindex].find("property") != string::npos) { codeindex++; continue; }
		if (plycode[codeindex].find("commit") != string::npos) { codeindex++; continue; }
		if (plycode[codeindex].find("element vertex") != string::npos) {
			vector<string> tokens;
			split(plycode[codeindex], tokens);
			vertexnum = atoi(tokens[2].c_str());
			codeindex++;
			continue;
		}
		if (plycode[codeindex].find("element face") != string::npos) {
			vector<string> tokens;
			split(plycode[codeindex], tokens);
			facenum = atoi(tokens[2].c_str());
			codeindex++;
			continue;
		}
		codeindex++;
	}
	codeindex++;
	glm::vec3 position;
	glm::vec3 color;
	glm::vec3 normal;
	for (unsigned int i = 0; i < vertexnum; i++) {
		vector<float> tokens;
		tokens = stringToNum<float>(plycode[codeindex++]);
		position = glm::vec3(tokens[0], tokens[1], tokens[2]);
		normal = glm::vec3(tokens[3], tokens[4], tokens[5]);
		color = glm::vec3(tokens[6] / 255, tokens[7] / 255, tokens[8] / 255);
		vertices.push_back({ position, normal, color });
	}
	for (unsigned int i = 0; i < facenum; i++) {
		vector<unsigned int> tokens;
		tokens = stringToNum<unsigned int>(plycode[codeindex++]);
		indices.push_back(tokens[1]);
		indices.push_back(tokens[2]);
		indices.push_back(tokens[3]);
	}
	Meshinfo meshinfo;
	meshes.push_back(Mesh(vertices, indices, meshinfo));
}
