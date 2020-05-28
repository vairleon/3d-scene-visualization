#ifndef PLANEFIT_H
#define PLANEFIT_H

#include "common.hpp"


class FitPlane{

public:
	
	FitPlane(PointCloudT::Ptr cloud_tgt);
	void ransacFitPlane();
	void planeAlignment_rough(const Eigen::Vector3f tgtvec3); // align pointclouds to xz-plane
	void planeAlignment();
	void planeRefinement();

	// settings
	void setSVDcoeff();  //set coeff to SVD of the plane
	void setConvexHullPolygon();

	Eigen::VectorXf getCoeff();
	Eigen::Vector3f getRotationAxis();
	float getRotationAngle();
	Eigen::Vector3f getTranslationVec3f();
	Eigen::Matrix4f getTransformationMatrix();
	void getCloudInliers(PointCloudT & cloud_inliers);

private:

	PointCloudT::Ptr cloud_inliers;
	PointCloudT::Ptr cloud_tgt; // 
	Eigen::VectorXf coeff;  // plane params
	Eigen::Vector3f rotationAxis;
	float rotationAngle;
	Eigen::Vector3f translation;
	Eigen::Matrix4f transformationMatrix;
	Eigen::Vector3f tgt_centroid;
	std::vector<int> best_model_3_point_indices;

	void RTtoTranformationMatrix();
};


#endif