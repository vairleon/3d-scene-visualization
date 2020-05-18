#ifndef UTILS_H
#define UTILS_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <regex>
#include <cstdlib>
#include <cstring>
#include <random>
#include <ctime>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h> 

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <Eigen/Core>

#ifndef NAMESPACE_STD
#define NAMESPACE_STD
using namespace std;
#endif

template <class Type>
vector<Type> stringToNum(const string& str)
{
	std::istringstream iss(str);
	Type num;
	vector<Type> vnum;
	while (iss >> num) {
		vnum.push_back(num);
	}
	return vnum;
}

void split(const string& s, vector<string>& tokens, const string& delimiters = " ");

glm::mat4 EigenMatrix4fToGlmMat4(const Eigen::Matrix4f& matrix);



struct Vx {
    float x, y, z;
    float padding;
    Vx operator-(const Vx& v)
    {
        Vx vt;
        vt.x = this->x - v.x;
        vt.y = this->y - v.y;
        vt.z = this->z - v.z;
        return vt;
    }
    Vx operator+(const Vx& v)
    {
        Vx vt;
        vt.x = this->x + v.x;
        vt.y = this->y + v.y;
        vt.z = this->z + v.z;
        return vt;
    }
    Vx operator*(const float& v)
    {
        Vx vt;
        vt.x = this->x * v;
        vt.y = this->y * v;
        vt.z = this->z * v;
        return vt;
    }
    Vx operator/(const float& v)
    {
        Vx vt;
        vt.x = this->x / v;
        vt.y = this->y / v;
        vt.z = this->z / v;
        return vt;
    }
};


void toPointCloud(const string objfilepath, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
//vector<string> fileReader(const string path);
//void objParserPointCloud(const vector<string>& objcode, vector<Vx>& vertices);



#endif