#include "utils.h"

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

glm::mat4 EigenMatrix4fToGlmMat4(const Eigen::Matrix4f& matrix) 
{
	glm::mat4 mat = glm::mat4(1.0f);
	for (int i = 0; i < 4; i++) {
		mat[i].x = matrix(0, i);
		mat[i].y = matrix(1, i);
		mat[i].z = matrix(2, i);
		mat[i].w = matrix(3, i);
	}
	return mat;
}

#define INTERPOLATION 5000
void objParserPointCloud(const vector<string>& objcode, vector<Vx>& vertices) {

	vector<unsigned int>  v_indices;

	unsigned int codeindex = 0;

	while (codeindex < objcode.size()) {
		std::istringstream iss(objcode[codeindex]);
		string token;
		iss >> token;
		if (token == "v") {
			vector<float> vnum;
			float num = 0;
			while (iss >> num) { vnum.push_back(num); }

			Vx vertex;
			vertex.x = vnum[0];
			vertex.y = vnum[1];
			vertex.z = vnum[2];
			vertices.push_back(vertex);
		}
		if (token == "f") {
			vector<string> tokens;
			split(objcode[codeindex], tokens);

			vector<unsigned int> temp_v_index;
			//vector<unsigned int> temp_vt_index;
			for (int i = 1; i < tokens.size(); i++) {
				vector<string> str_index;
				split(tokens[i], str_index, "/");
				temp_v_index.push_back((unsigned int)atoi(str_index[0].c_str()) - 1);
			}

			// S_triangle =(1/2)*(x1y2+x2y3+x3y1-x1y3-x2y1-x3y2)  calculate triangle area....
			// cross(u, v)=（Yu*Zv–Zu*Yv）*i+（Zu*Xv–Xu*Zv）*j+（Xu*Yv–Yu*Xv）*k。
			Vx a = vertices[temp_v_index[0]];
			Vx b = vertices[temp_v_index[1]];
			Vx c = vertices[temp_v_index[2]];
			Vx u = b - a;
			Vx v = c - a;
			float area = sqrt(pow(u.y*v.z - u.z*v.y, 2) + pow(u.z*v.x - u.x*v.z, 2) + pow(u.x*v.y - u.y*v.x, 2));
			int ipnum = (int)(INTERPOLATION * area) ;

			
			random_device rd;
			default_random_engine e{rd()};
			uniform_real_distribution<float> ud(0, 1); //随机数分布对象 
			for (int i = 0; i < ipnum; i++) { //对三角面片内部点随机采样并加入点云
				//srand((int)time(0));
				float f = ud(e);
				float g = ud(e);
				float h = ud(e);
				Vx v = (a * f + b * g + c * h) / (f + g + h);
				vertices.push_back(v);
			}
			// interpolate points randomly within this 'area'


		}
		//else
		codeindex++;
	}


	return;
}

vector<string> fileReader(const string path) {

	// 从文件路径中获取模型文件
	vector<string> codes; //读文档
	std::ifstream modelFile;
	// 保证ifstream对象可以抛出异常：
	modelFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
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

	return codes;
}

void toPointCloud(const string objfilepath, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) {
	vector<string> objcodes = fileReader(objfilepath);
	vector<Vx> vertices_obj;
	objParserPointCloud(objcodes, vertices_obj);
	for (unsigned int i = 0; i < vertices_obj.size(); i++) {
		(*cloud).push_back(pcl::PointXYZ(vertices_obj[i].x, vertices_obj[i].y, vertices_obj[i].z));
	}
}