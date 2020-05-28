#include "model.h"

template <class Type>
vector<Type> stringToNum(const string& str);
static void split(const string& s, vector<string>& tokens, const string& delimiters = " ");
static void plyParser(const vector<string>& plycode, vector<Vertex>& vertexs, vector<unsigned int>& indices);

Model::Model(vector<string> path) {
	for (unsigned int i = 0; i < path.size(); i++) {
		Mesh mesh = loadMesh(path[i]);
		meshes.push_back(mesh);
	}
}

void Model::Draw(Shader shader) {
    for (unsigned int i = 0; i < meshes.size(); i++)
        meshes[i].Draw(shader);
}

void Model::loadModel(string path) {
    return ;
}

Mesh Model::loadMesh(string path) {

	// 从文件路径中获取.ply文件
	vector<std::string> plyCode; //读文档
	vector<unsigned int> indices;
	vector<Vertex> vertexs;
	std::ifstream PlyFile;
	// 保证ifstream对象可以抛出异常：
	PlyFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try
	{
		// 打开文件
		string code;
		PlyFile.open(path, std::ifstream::in);
		while (!PlyFile.eof()) {
			getline(PlyFile, code);
			plyCode.push_back(code);
		}
		PlyFile.close();
		// 转换数据流到string
		// plyCode = PlyStream.str();
	}
	catch (std::ifstream::failure e)
	{
		std::cout << "ERROR::MODEL_PLYFILE::FILE_NOT_SUCCESFULLY_READ" << std::endl;
	}

	// 利用plyCode创建Mesh对象
	plyParser(plyCode, vertexs, indices);
	Mesh mesh = Mesh(vertexs, indices);
	return mesh;
}
// plyParser 只解析固定格式的plyfile 
static void plyParser(const vector<string>& plycode, vector<Vertex>& vertexs, vector<unsigned int>& indices) {
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
	for (unsigned int i = 0; i < vertexnum; i++) {
		vector<float> tokens;
		tokens = stringToNum<float>(plycode[codeindex++]);
		position = glm::vec3(tokens[0], tokens[1], tokens[2]);
		color = glm::vec3(tokens[3]/255, tokens[4]/255, tokens[5]/255);
		vertexs.push_back({ position, color });
	}
	for (unsigned int i = 0; i < facenum; i++) {
		vector<unsigned int> tokens;
		tokens = stringToNum<unsigned int>(plycode[codeindex++]);
		indices.push_back(tokens[1]);
		indices.push_back(tokens[2]);
		indices.push_back(tokens[3]);
	}

}

void split(const string& s, vector<string>& tokens, const string& delimiters)
{
	string::size_type lastPos = s.find_first_not_of(delimiters, 0);
	string::size_type pos = s.find_first_of(delimiters, lastPos);
	while (string::npos != pos || string::npos != lastPos) {
		tokens.push_back(s.substr(lastPos, pos - lastPos));
		lastPos = s.find_first_not_of(delimiters, pos);
		pos = s.find_first_of(delimiters, lastPos);
	}
}

template <class Type>
vector<Type> stringToNum(const string& str)
{
	istringstream iss(str);
	Type num;
	vector<Type> vnum;
	while (iss >> num) {
		vnum.push_back(num);
	}
	return vnum;
}