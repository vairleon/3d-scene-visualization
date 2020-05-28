#include "filters.h"



void voxelFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out, float gridsize){
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setLeafSize(gridsize, gridsize, gridsize);
	vox_grid.setInputCloud(cloud_in);
	vox_grid.filter(*cloud_out);
	return;
}

void passFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, double filter_limit){
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(pc);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(0, filter_limit);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0, filter_limit);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, filter_limit);
	pass.filter(*pc);
	return;
}


