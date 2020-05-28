#ifndef CADDB_H
#define CADDB_H


#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h> 
#include "utils/utils.h"

std::string query(pcl::PointCloud<pcl::PointXYZ>::Ptr query_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud);

#endif