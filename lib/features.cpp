#include "features.h"



pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double features_radius) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method_fpfh(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_est.setInputCloud(cloud);
	fpfh_est.setInputNormals(normals);
	fpfh_est.setSearchMethod(search_method_fpfh);
	fpfh_est.setRadiusSearch(features_radius);
	fpfh_est.compute(*fpfh_features);
	return fpfh_features;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int features_K) {
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr search_method_fpfh(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh_features(new pcl::PointCloud<pcl::FPFHSignature33>());
	fpfh_est.setInputCloud(cloud);
	fpfh_est.setInputNormals(normals);
	fpfh_est.setSearchMethod(search_method_fpfh);
	fpfh_est.setKSearch(features_K);
	fpfh_est.compute(*fpfh_features);
	return fpfh_features;
}


pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, double normals_radius = 0.25) {

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree< pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree< pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud< pcl::Normal>);
	ne.setInputCloud(incloud);
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(normals_radius);
	ne.compute(*cloud_normals);
	return cloud_normals;
}
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, int normals_K){

	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::search::KdTree< pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree< pcl::PointXYZ>());
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud< pcl::Normal>);
	ne.setInputCloud(incloud);
	ne.setSearchMethod(kdtree);
	ne.setKSearch(normals_K);
	ne.compute(*cloud_normals);
	return cloud_normals;

}
