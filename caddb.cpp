#include "caddb.h"

std::string query(pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud) {
	std::string path = "output\\temp\\1b3c286bce219ddb7135fc51795b4038\\models\\model_normalized.obj";
	toPointCloud(path, result_cloud);
	/* return Path */
	return path;
}