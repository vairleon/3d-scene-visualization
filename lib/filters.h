#ifndef LIBFILTERS_H
#define LIBFILTERS_H

#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h> 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

using namespace std;

void voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float gridsize = 2.0f);
void passFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double filter_limit = 1000.0);

#endif