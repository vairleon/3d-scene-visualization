#ifndef LIBFEATURE_H
#define LIBFEATURE_H

#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h> 
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>


using namespace std;

pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double features_radius = 0.25);
pcl::PointCloud<pcl::FPFHSignature33>::Ptr getFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, int features_K);
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, double normals_radius);
pcl::PointCloud<pcl::Normal>::Ptr getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, int normals_K);

#endif