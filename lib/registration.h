#ifndef REGISTRATION_H
#define REGISTRATION_H

#include "common.hpp"
#include "sac_ia.h"
#include "filters.h"
#include "icp_fixy.hpp"

class Registration {

public:

    Registration();
	Registration(PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_tgt);
    ~Registration();

    void setInputCloud(PointCloudT::Ptr cloud_src);
    void setTargetCloud(PointCloudT::Ptr cloud_tgt);
	void regMethodBySacIaAndIcp(); /* method 1 -> 不对scale回归（假设scale可以通过计算最小包围盒获取）*/
	void regMethodByBruteAlignAndIcp(); /* plan B -> use brute alignment methods for the first step */
	void regMethodBySacIaAndIcpWithAlignedFloor(); /* improved sac-ia & icp */


	Eigen::Matrix4f getFinalTransformation();

private:
	PointCloudT::Ptr cloud_src;
	PointCloudT::Ptr cloud_tgt;
	Eigen::Matrix4f transformation_matrix;

    void setupCloud();
	void _transformation(const Eigen::Matrix4f& matrix_in);
	

};
#endif