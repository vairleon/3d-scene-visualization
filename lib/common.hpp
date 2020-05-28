
#ifndef COMMOM_H
#define COMMOM_H

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
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// sac - params
const float VOXEL_GRID_SIZE = 0.1f; // basic

const float VOXEL_GRID_SIZE_Level_1 = 0.1f;  //sac_ia
const float VOXEL_GRID_SIZE_Level_2 = 0.05f;  //icp


// not used ...
const int NORMAL_SEARCH_K = 15;
const float FPFH_SEARCH_RADIUS = 0.25f;
const int Correspondence_Randomness = 5;
const int Number_OF_Samples = 5;
const float Min_Sample_Distance = 0.1f;

// icp - params
const int ICP_MAX_ITERATION = 20;
const float ICP_TRANS_EPSILON = 1e-10;
const float ICP_MAX_CORR_DISTANCE = 0.3f;

// planefit ransac

const float RANSAC_DISTANCE_THRESHOLD = 0.025;



// funcs 

template <typename PointSource, typename PointTarget, typename Scalar> void
getTransformationFromCorrelationFixY(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_src_demean,
    const Eigen::Matrix<Scalar, 3, 1>& centroid_src,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_tgt_demean,
    const Eigen::Matrix<Scalar, 3, 1>& centroid_tgt,
    Eigen::Matrix4f& transformation_matrix);

template <typename PointSource, typename PointTarget, typename Scalar> void
estimateRigidTransformationSVD(
    const pcl::PointCloud<PointSource>& cloud_src,
    const std::vector<int>& indices_src,
    const pcl::PointCloud<PointTarget>& cloud_tgt,
    const std::vector<int>& indices_tgt,
    Eigen::Matrix4f& transformation_matrix);

template <typename PointSource, typename PointTarget, typename Scalar> void
estimateRigidTransformation(
    pcl::ConstCloudIterator<PointSource>& source_it,
    pcl::ConstCloudIterator<PointTarget>& target_it,
    Eigen::Matrix4f& transformation_matrix);


template <typename PointSource, typename PointTarget, typename Scalar> void
estimateRigidTransformation(
    const pcl::PointCloud<PointSource>& cloud_src,
    const pcl::PointCloud<PointTarget>& cloud_tgt,
    const pcl::Correspondences& correspondences,
    Eigen::Matrix4f& transformation_matrix);

template <typename PointSource, typename PointTarget, typename Scalar> void
estimateRigidTransformation(
    const pcl::PointCloud<PointSource>& cloud_src,
    const pcl::PointCloud<PointTarget>& cloud_tgt,
    const pcl::Correspondences& correspondences,
    Eigen::Matrix4f& transformation_matrix)
{
    pcl::ConstCloudIterator<PointSource> source_it(cloud_src, correspondences, true);
    pcl::ConstCloudIterator<PointTarget> target_it(cloud_tgt, correspondences, false);
    estimateRigidTransformation<PointSource, PointTarget, Scalar>(source_it, target_it, transformation_matrix);
}


template <typename PointSource, typename PointTarget, typename Scalar> void
estimateRigidTransformationSVD(
    const pcl::PointCloud<PointSource>& cloud_src,
    const std::vector<int>& indices_src,
    const pcl::PointCloud<PointTarget>& cloud_tgt,
    const std::vector<int>& indices_tgt,
    Eigen::Matrix4f& transformation_matrix)
{
    if (indices_src.size() != indices_tgt.size())
    {
        PCL_ERROR("[pcl::TransformationEstimationSVD::estimateRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size(), indices_tgt.size());
        return;
    }

    pcl::ConstCloudIterator<PointSource> source_it(cloud_src, indices_src);
    pcl::ConstCloudIterator<PointTarget> target_it(cloud_tgt, indices_tgt);
    estimateRigidTransformation<PointSource, PointTarget, Scalar>(source_it, target_it, transformation_matrix);
}


template <typename PointSource, typename PointTarget, typename Scalar> void
estimateRigidTransformation(
    pcl::ConstCloudIterator<PointSource>& source_it,
    pcl::ConstCloudIterator<PointTarget>& target_it,
    Eigen::Matrix4f& transformation_matrix)
{
    // Convert to Eigen format
    const int npts = static_cast <int> (source_it.size());

    if (false)
    {
        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_src(3, npts);
        Eigen::Matrix<Scalar, 3, Eigen::Dynamic> cloud_tgt(3, npts);

        for (int i = 0; i < npts; ++i)
        {
            cloud_src(0, i) = source_it->x;
            cloud_src(1, i) = source_it->y;
            cloud_src(2, i) = source_it->z;
            ++source_it;

            cloud_tgt(0, i) = target_it->x;
            cloud_tgt(1, i) = target_it->y;
            cloud_tgt(2, i) = target_it->z;
            ++target_it;
        }

        // Call Umeyama directly from Eigen (PCL patched version until Eigen is released)
        transformation_matrix = pcl::umeyama(cloud_src, cloud_tgt, false);
    }
    else
    {
        // Convert to Eigen format
        const int npts = static_cast <int> (source_it.size());

        source_it.reset();
        target_it.reset();
        // <cloud_src,cloud_src> is the source dataset
        transformation_matrix.setIdentity();

        Eigen::Matrix<Scalar, 4, 1> centroid_src, centroid_tgt;
        // Estimate the centroids of source, target
        pcl::compute3DCentroid(source_it, centroid_src);
        pcl::compute3DCentroid(target_it, centroid_tgt);
        source_it.reset();
        target_it.reset();

        // Subtract the centroids from source, target
        Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> cloud_src_demean, cloud_tgt_demean;
        pcl::demeanPointCloud(source_it, centroid_src, cloud_src_demean);
        pcl::demeanPointCloud(target_it, centroid_tgt, cloud_tgt_demean);


        // fix y axis
        Eigen::Matrix<Scalar, 3, 1> centroid_src_fy = Eigen::Matrix<Scalar, 3, 1>(centroid_src[0], centroid_src[2], centroid_src[3]);
        Eigen::Matrix<Scalar, 3, 1> centroid_tgt_fy = Eigen::Matrix<Scalar, 3, 1>(centroid_tgt[0], centroid_tgt[2], centroid_tgt[3]);


        // remove y data
        unsigned int numRows = cloud_src_demean.rows() - 1;
        unsigned int numCols = cloud_tgt_demean.cols();

        //#include <iostream>
        //std::cout << cloud_src_demean << std::endl;
        //std::cout << cloud_tgt_demean << std::endl;
        cloud_src_demean.block(1, 0, numRows - 1, numCols) = cloud_src_demean.block(2, 0, numRows - 1, numCols);
        cloud_src_demean.conservativeResize(numRows, numCols);
        cloud_tgt_demean.block(1, 0, numRows - 1, numCols) = cloud_tgt_demean.block(2, 0, numRows - 1, numCols);
        cloud_tgt_demean.conservativeResize(numRows, numCols);
        //std::cout  << std::endl;
        //std::cout << cloud_src_demean << std::endl;
        //std::cout << cloud_tgt_demean << std::endl;
        getTransformationFromCorrelationFixY<PointSource, PointTarget, Scalar>(cloud_src_demean, centroid_src_fy, cloud_tgt_demean, centroid_tgt_fy, transformation_matrix);


        //getTransformationFromCorrelationFixY<PointSource, PointTarget, Scalar>(cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, transformation_matrix);

    }

}


template <typename PointSource, typename PointTarget, typename Scalar> void
getTransformationFromCorrelationFixY(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_src_demean,
    const Eigen::Matrix<Scalar, 4, 1>& centroid_src,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_tgt_demean,
    const Eigen::Matrix<Scalar, 4, 1>& centroid_tgt,
    Eigen::Matrix4f& transformation_matrix)
{

    transformation_matrix.setIdentity();

    // Assemble the correlation matrix H = source * target'
    Eigen::Matrix<Scalar, 3, 3> H = (cloud_src_demean * cloud_tgt_demean.transpose()).topLeftCorner(3, 3);

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 3, 3> u = svd.matrixU();
    Eigen::Matrix<Scalar, 3, 3> v = svd.matrixV();

    // Compute R = V * U'
    if (u.determinant() * v.determinant() < 0)
    {
        for (int x = 0; x < 3; ++x)
            v(x, 2) *= -1;
    }

    Eigen::Matrix<Scalar, 3, 3> R = v * u.transpose();

    // Return the correct transformation
    transformation_matrix.topLeftCorner(3, 3) = R;
    const Eigen::Matrix<Scalar, 3, 1> Rc(R * centroid_src.head(3));
    transformation_matrix.block(0, 3, 3, 1) = centroid_tgt.head(3) - Rc;
}



template <typename PointSource, typename PointTarget, typename Scalar> void
getTransformationFromCorrelationFixY(
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_src_demean,
    const Eigen::Matrix<Scalar, 3, 1>& centroid_src,
    const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>& cloud_tgt_demean,
    const Eigen::Matrix<Scalar, 3, 1>& centroid_tgt,
    Eigen::Matrix4f& transformation_matrix)
{

    transformation_matrix.setIdentity();


    // Assemble the correlation matrix H = source * target'
    Eigen::Matrix<Scalar, 2, 2> H = (cloud_src_demean * cloud_tgt_demean.transpose()).topLeftCorner(2, 2);

    // Compute the Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2, 2> > svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix<Scalar, 2, 2> u = svd.matrixU();
    Eigen::Matrix<Scalar, 2, 2> v = svd.matrixV();

    // Compute R = V * U'
    if (u.determinant() * v.determinant() < 0)
    {
        for (int x = 0; x < 2; ++x)
            v(x, 1) *= -1;
    }

    Eigen::Matrix<Scalar, 2, 2> R = v * u.transpose();

    // Return the correct transformation
    transformation_matrix(0, 0) = R(0, 0);
    transformation_matrix(0, 2) = R(0, 1);
    transformation_matrix(2, 0) = R(1, 0);
    transformation_matrix(2, 2) = R(1, 1);

    const Eigen::Matrix<Scalar, 2, 1> Rc(centroid_tgt.head(2) - R * centroid_src.head(2));
    transformation_matrix(0, 3) = Rc[0];
    transformation_matrix(2, 3) = Rc[1];
}



#endif