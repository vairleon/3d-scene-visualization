#include "planefit.h"

FitPlane::FitPlane(PointCloudT::Ptr cloud_tgt)
{
    this->cloud_tgt = cloud_tgt;
    this->cloud_inliers = PointCloudT::Ptr(new PointCloudT);
    coeff = Eigen::VectorXf(4);

    // calculate centroid of target pointcloud
    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid(*cloud_tgt, centroid4f);
    Eigen::Vector3f tempcentroid(centroid4f[0], centroid4f[1], centroid4f[2]);
    this->tgt_centroid = tempcentroid;
}

void FitPlane::ransacFitPlane() 
{
 
    std::vector<int> inliers; // 下标
    pcl::SampleConsensusModelPlane<PointT>::Ptr model(
        new pcl::SampleConsensusModelPlane<PointT>(cloud_tgt));//定义待拟合平面的model，并使用待拟合点云初始化
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model); //定义RANSAC算法模型
    ransac.setDistanceThreshold(RANSAC_DISTANCE_THRESHOLD);//设定阈值
    ransac.computeModel(); //拟合
    ransac.getInliers(inliers);//获取合群点

    ransac.getModelCoefficients(coeff);//获取拟合平面参数，对于平面ax+by_cz_d=0，coeff分别按顺序保存a,b,c,d
    ransac.getModel(best_model_3_point_indices);

    pcl::copyPointCloud(*cloud_tgt, inliers, *cloud_inliers); //
}

//void FitPlane::setSVDcoeff()
//{
//    
//}


void FitPlane::planeAlignment() 
{

    float dist = fabs(coeff[3]) / sqrt(pow(coeff[0], 2) + pow(coeff[1], 2) + pow(coeff[2], 2));

    std::vector<Eigen::Vector3f> v;
    
    for (int i = 0; i < 2; i++) {
        int index = best_model_3_point_indices[i];
        float _z = 0, _x = (*cloud_tgt)[index].x, _y = (*cloud_tgt)[index].y;
        _z = -(coeff[0] * _x + coeff[1] * _y + coeff[3]) / coeff[2];
        v.push_back(Eigen::Vector3f(_x, _y, _z));
    }
    
    Eigen::Vector3f tangent = v[0] - v[1];
    Eigen::Vector3f normal = Eigen::Vector3f(coeff[0], coeff[1], coeff[2]);
    tangent.normalize();
    normal.normalize();

    Eigen::Vector3f judgedirection = tgt_centroid - v[0];
    normal = (normal.dot(judgedirection) > 0.0f) ? normal : normal * (-1);
    Eigen::Vector3f bitangent = normal.cross(tangent);
    bitangent.normalize();

    this->translation = -dist * normal;
    Eigen::Matrix3f rotationMatrix = Eigen::Matrix3f::Identity();

    for (int i = 0; i < 3; i++) {
        rotationMatrix(0, i) = tangent(i);
        rotationMatrix(1, i) = normal(i);
        rotationMatrix(2, i) = bitangent(i);
    }

    transformationMatrix = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++) {
        transformationMatrix(i, 0) = rotationMatrix(i, 0);
        transformationMatrix(i, 1) = rotationMatrix(i, 1);
        transformationMatrix(i, 2) = rotationMatrix(i, 2);
        transformationMatrix(i, 3) = this->translation[i];
    }
}

void FitPlane::planeAlignment_rough(const Eigen::Vector3f tgtvec3)
{
    // get best-fit plane coeff
    // setSVDcoeff();

    Eigen::Vector3f tgtvec3f = tgtvec3;
    tgtvec3f.normalize();

    // distance from origin to plane
    float dist = fabs(coeff[3]) / sqrt(pow(coeff[0], 2) + pow(coeff[1], 2) + pow(coeff[2], 2));

    // get normal of the plane
    Eigen::Vector3f normal = Eigen::Vector3f(coeff[0], coeff[1], coeff[2]);
    normal.normalize();
    
    // 防止数值下溢 (校正法向量误差)
    if (normal.norm() < 0.0005f) {
        // huge problem
    }

    // set plane direction
    normal =  (normal.dot(tgt_centroid) > 0.0f) ? normal : normal * (-1);
    // set transformation variables
    this->translation = -dist * normal;
    Eigen::Vector3f rotvec3f = tgtvec3f.cross(normal);
    rotvec3f.normalize();
    float cos_angle = tgtvec3f.dot(normal) / (tgtvec3f.norm() * normal.norm());
    
    this->rotationAngle = acos(cos_angle) * 180 / M_PI;
    this->rotationAxis = Eigen::Vector3f(rotvec3f);

    RTtoTranformationMatrix();
}

Eigen::Vector3f FitPlane::getRotationAxis() 
{
    return this->rotationAxis;
}

float FitPlane::getRotationAngle()
{
    return this->rotationAngle;
}

Eigen::Vector3f FitPlane::getTranslationVec3f() 
{
    return this->translation;
}

void FitPlane::getCloudInliers(PointCloudT& cloud_inliers)
{
    pcl::copyPointCloud(*(this->cloud_inliers), cloud_inliers);
}

Eigen::VectorXf FitPlane::getCoeff() {
    return this->coeff;
}

Eigen::Matrix4f FitPlane::getTransformationMatrix()
{
    return transformationMatrix;
}

void FitPlane::RTtoTranformationMatrix()
{
    Eigen::AngleAxisf rotationAX(this->rotationAngle, this->rotationAxis);
    Eigen::Matrix3f rotationMatrix = rotationAX.matrix();
    
    transformationMatrix = Eigen::Matrix4f::Identity();

    for (int i = 0; i < 3; i++) {
        transformationMatrix(i, 0) = rotationMatrix(i, 0);
        transformationMatrix(i, 1) = rotationMatrix(i, 1);
        transformationMatrix(i, 2) = rotationMatrix(i, 2);
        transformationMatrix(i, 3) = this->translation[i];
    }
}