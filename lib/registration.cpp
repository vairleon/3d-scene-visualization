#include "registration.h"

void
print4x4Matrix(const Eigen::Matrix4f& matrix)
{
	printf("Rotation matrix :\n");
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
	printf("Translation vector :\n");
	printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix(0, 3), matrix(1, 3), matrix(2, 3));
}

Registration::Registration() : transformation_matrix(Eigen::Matrix4f::Identity()) {}
Registration::~Registration() {}

Registration::Registration(PointCloudT::Ptr cloud_src, PointCloudT::Ptr cloud_tgt) : transformation_matrix(Eigen::Matrix4f::Identity()) {
	this->cloud_src = cloud_src;
	this->cloud_tgt = cloud_tgt;
}


void Registration::setInputCloud(PointCloudT::Ptr cloud_src) {
	this->cloud_src = cloud_src;
}

void Registration::setTargetCloud(PointCloudT::Ptr cloud_tgt) {
	this->cloud_tgt = cloud_tgt;
}

Eigen::Matrix4f Registration::getFinalTransformation() {
	return this->transformation_matrix;
}

void Registration::setupCloud() {
	if ((*cloud_src).width * (*cloud_tgt).width * (*cloud_src).height * (*cloud_src).height == 0) {
		cout << "Error: Cannot setup pointclouds, since input cloud or target cloud was not set!" << endl;
	}
}

void Registration::regMethodBySacIaAndIcp() {
	if ((*cloud_src).width * (*cloud_tgt).width * (*cloud_src).height * (*cloud_src).height == 0) {
		cout << "Error: Cannot setup pointclouds, since input cloud or target cloud was not set!" << endl;
		return;
	}
	cout << "Perform regMethodBySacIaAndIcp ..." << endl;

	Eigen::Matrix4f temp_trans = Eigen::Matrix4f::Identity();

	// 求包围盒与质心 
	float scale = 1.0f;
	Eigen::Vector3f translate;
	// ---- PCA based method -----
	// ...

	// ---- bounding box ---- 
	PointT min_src, max_src, min_tgt, max_tgt;
	Eigen::Vector3f c_src, c_tgt;
	pcl::getMinMax3D(*cloud_src, min_src, max_src);
	pcl::getMinMax3D(*cloud_tgt, min_tgt, max_tgt);
	c_src = 0.5f * (min_src.getVector3fMap() + max_src.getVector3fMap());
	c_tgt = 0.5f * (min_tgt.getVector3fMap() + max_tgt.getVector3fMap());
	scale = (max_tgt.z - min_tgt.z) / (max_src.z - min_src.z);  // the floor of scene has supposed to been aligned 
	translate = c_tgt - c_src;

	// A rotation matrix 
	float theta = 0;  // The angle of rotation in radians
	temp_trans(0, 0) = std::cos(theta) * scale;
	//transformation_matrix(0, 1) = -sin(theta) * scale;
	//transformation_matrix(1, 0) = sin(theta) * scale;
	temp_trans(1, 1) = std::cos(theta) * scale;
	temp_trans(2, 2) = scale;
	// A translation 
	temp_trans(0, 3) = translate[0];
	temp_trans(1, 3) = translate[1];
	temp_trans(2, 3) = translate[2];

	// implement temp_trans to final transformation_matrix
	_transformation(temp_trans);

	
	// ---- further Alignment ----- 

	// Normalization  -  为了适配sac-ia配准参数
	PointCloudT::Ptr cloud_src_nl(new PointCloudT);
	PointCloudT::Ptr cloud_tgt_nl(new PointCloudT);

	temp_trans = Eigen::Matrix4f::Identity();
	// A rotation matrix 
	float nscale = scale / (max_tgt.z - c_tgt[2] > c_tgt[2] - min_tgt.z ? max_tgt.z - c_tgt[2] : c_tgt[2] - min_tgt.z);
	theta = 0;  // The angle of rotation in radians
	// A translation 
	temp_trans(0, 3) = -c_src[0];
	temp_trans(1, 3) = -c_src[1];
	temp_trans(2, 3) = -c_src[2];

	pcl::transformPointCloud(*cloud_src, *cloud_src_nl, temp_trans);

	temp_trans = Eigen::Matrix4f::Identity();
	// A rotation matrix 
	theta = 0;  // The angle of rotation in radians
	temp_trans(0, 0) = std::cos(theta) * nscale;
	temp_trans(1, 1) = std::cos(theta) * nscale;
	temp_trans(2, 2) = nscale;

	pcl::transformPointCloud(*cloud_src_nl, *cloud_src_nl, temp_trans);
	//pcl::getMinMax3D(*cloud_src_nl, min_src, max_src);


	temp_trans = Eigen::Matrix4f::Identity();
	// A rotation matrix 
	nscale = 1 / (max_tgt.z - c_tgt[2] > c_tgt[2] - min_tgt.z ? max_tgt.z - c_tgt[2] : c_tgt[2] - min_tgt.z);
	theta = 0;  // The angle of rotation in radians
	// A translation 
	temp_trans(0, 3) = -c_tgt[0];
	temp_trans(1, 3) = -c_tgt[1];
	temp_trans(2, 3) = -c_tgt[2];

	pcl::transformPointCloud(*cloud_tgt, *cloud_tgt_nl, temp_trans);
	temp_trans = Eigen::Matrix4f::Identity();
	// A rotation matrix 
	theta = 0;  // The angle of rotation in radians
	temp_trans(0, 0) = std::cos(theta) * nscale;
	temp_trans(1, 1) = std::cos(theta) * nscale;
	temp_trans(2, 2) = nscale;

	pcl::transformPointCloud(*cloud_tgt_nl, *cloud_tgt_nl, temp_trans);
	//pcl::getMinMax3D(*cloud_tgt_nl, min_tgt, max_tgt);

	// downsampling
	PointCloudT::Ptr tempCloud_src(new PointCloudT);
	voxelFilter(cloud_src_nl, tempCloud_src, VOXEL_GRID_SIZE);
	cloud_src_nl = tempCloud_src;
	PointCloudT::Ptr tempCloud_tgt(new PointCloudT);
	voxelFilter(cloud_tgt_nl, tempCloud_tgt, VOXEL_GRID_SIZE);
	cloud_tgt_nl = tempCloud_tgt;

	// sac_ia align
	PointCloudT::Ptr cloud_sac_result(new PointCloudT);
	temp_trans = Eigen::Matrix4f::Identity();
	temp_trans = sac_algin(cloud_src_nl, cloud_tgt_nl, cloud_sac_result, NORMAL_SEARCH_K, FPFH_SEARCH_RADIUS);
	
	// implement temp_trans to final transformation_matrix
	_transformation(temp_trans);


	
	// icp align

	PointCloudT::Ptr cloud_icp_result(new PointCloudT);
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaximumIterations(ICP_MAX_ITERATION);
	//icp.setMaxCorrespondenceDistance(ICP_MAX_CORR_DISTANCE);
	//icp.setTransformationEpsilon(ICP_TRANS_EPSILON);
	//icp.setEuclideanFitnessEpsilon(0.2);
	icp.setInputSource(cloud_sac_result);
	icp.setInputTarget(cloud_tgt_nl);
	icp.align(*cloud_icp_result);

	temp_trans = Eigen::Matrix4f::Identity();
	temp_trans = icp.getFinalTransformation();
	_transformation(temp_trans);
	/*
	*/
	/*cloud_src->~PointCloudT();
	cloud_tgt->~PointCloudT();
	cloud_src_nl->~PointCloudT();
	cloud_tgt_nl->~PointCloudT();*/
}

void Registration::_transformation(const Eigen::Matrix4f& matrix_in) {
	Eigen::Matrix3f rotation_in = Eigen::Matrix3f::Identity();
	Eigen::Vector3f translation_in = Eigen::Vector3f::Identity();
	
	Eigen::Matrix3f rotation = Eigen::Matrix3f::Identity();
	Eigen::Vector3f translation = Eigen::Vector3f::Identity();

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			rotation_in(i, j) = matrix_in(i, j);
			rotation(i, j) = transformation_matrix(i, j);
		}
	}

	for (int i = 0; i < 3; i++) {
		translation_in[i] = matrix_in(i, 3);
		translation[i] = transformation_matrix(i, 3);
	}

	rotation = rotation_in * rotation;
	translation = translation_in + translation;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			transformation_matrix(i, j) = rotation(i, j);
		}
	}
	
	for (int i = 0; i < 3; i++) {
		transformation_matrix(i, 3) = translation[i];
	}

}
