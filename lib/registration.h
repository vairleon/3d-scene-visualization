#ifndef REGISTRATION_H
#define REGISTRATION_H

#include <limits>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include "common.hpp"
#include "sac_ia.h"
#include "filters.h"

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Registration {

public:

    Registration();
	Registration(PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_tgt);
    ~Registration();

    void setInputCloud(PointCloudT::Ptr cloud_src);
    void setTargetCloud(PointCloudT::Ptr cloud_tgt);
	void regMethodBySacIaAndIcp(); /* method 1 -> 不对scale回归（假设scale可以通过计算最小包围盒获取）*/
	void regMethodByBruteAlignAndIcp(); /* plan B -> use brute alignment methods for the first step */

	Eigen::Matrix4f getFinalTransformation();

private:
	PointCloudT::Ptr cloud_src;
	PointCloudT::Ptr cloud_tgt;
	Eigen::Matrix4f transformation_matrix;
    void setupCloud();
	void _transformation(const Eigen::Matrix4f& matrix_in);
	

};
#endif