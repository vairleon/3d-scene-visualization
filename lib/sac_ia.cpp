#include "sac_ia.h"
#include "ransac_fixy.hpp"


//SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33>
//
//align(PointCloud<PointXYZ>::Ptr cloud1, PointCloud<PointXYZ>::Ptr cloud2,
//PointCloud<FPFHSignature33>::Ptr features1, PointCloud<FPFHSignature33>::Ptr features2,
//int max_sacia_iterations, double min_correspondence_dist, double max_correspondence_dist) {
//
//	SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
//	Eigen::Matrix4f final_transformation;
//	sac_ia.setInputCloud(cloud2);
//	sac_ia.setSourceFeatures(features2);
//	sac_ia.setInputTarget(cloud1);
//	sac_ia.setTargetFeatures(features1);
//	sac_ia.setMaximumIterations(max_sacia_iterations);
//	sac_ia.setMinSampleDistance(min_correspondence_dist);
//	sac_ia.setMaxCorrespondenceDistance(max_correspondence_dist);
//	PointCloud<PointXYZ> finalcloud;
//	sac_ia.align(finalcloud);
//	sac_ia.getCorrespondenceRandomness();
//	return sac_ia;
//}

// SampleConsensusInitialAlignment<PointCloudT, PointCloudT, >;
Eigen::Matrix4f sac_align(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
	pcl::PointCloud<pcl::PointXYZ>::Ptr result, int normals_K, double features_radius) 
{
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;

	cloud_src_normals = getNormals(cloud_src, normals_K);
	cloud_tgt_normals = getNormals(cloud_tgt, normals_K);
	fpfhs_src = getFeatures(cloud_src, cloud_src_normals, features_radius);
	fpfhs_tgt = getFeatures(cloud_tgt, cloud_tgt_normals, features_radius);
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setMinSampleDistance(0.2);
	//scia.setNumberOfSamples(10);
	//scia.setCorrespondenceRandomness(5);
	scia.align(*result);
	return scia.getFinalTransformation();
}


Eigen::Matrix4f sac_fixy_align(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt,
	pcl::PointCloud<pcl::PointXYZ>::Ptr result) {
	pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud< pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud< pcl::Normal>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>());
	SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;

	cloud_src_normals = getNormals(cloud_src, NORMAL_SEARCH_K);
	cloud_tgt_normals = getNormals(cloud_tgt, NORMAL_SEARCH_K);
	fpfhs_src = getFeatures(cloud_src, cloud_src_normals, FPFH_SEARCH_RADIUS);
	fpfhs_tgt = getFeatures(cloud_tgt, cloud_tgt_normals, FPFH_SEARCH_RADIUS);
	scia.setInputSource(cloud_src);
	scia.setInputTarget(cloud_tgt);
	scia.setSourceFeatures(fpfhs_src);
	scia.setTargetFeatures(fpfhs_tgt);
	//scia.setCorrespondenceRandomness(Correspondence_Randomness);
	//scia.setNumberOfSamples(Number_OF_Samples);
	// scia.setMinSampleDistance(Min_Sample_Distance);
	scia.align(*result);
	return scia.getFinalTransformation();
}