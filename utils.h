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


#endif