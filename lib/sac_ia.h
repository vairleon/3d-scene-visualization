#ifndef LIBSAC_IA_H
#define LIBSAC_IA_H

#include <vector>
#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ia_ransac.h>

#include "features.h"

Eigen::Matrix4f sac_algin(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
	pcl::PointCloud<pcl::PointXYZ>::Ptr result, int normals_K, double features_radius);


#endif