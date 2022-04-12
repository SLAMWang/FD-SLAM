//
// Created by robot410 on 2020/11/3.
//

#ifndef LOAM_ODOMETRY_IMAGEPROJECTION_H
#define LOAM_ODOMETRY_IMAGEPROJECTION_H

#include "hdl_graph_slam/utility.h"
#include<unistd.h>
namespace hdl_graph_slam {
    typedef Eigen::Matrix<float, 64, 1> Vec16f;
    typedef Eigen::Matrix<float, 21, 1> Vector21f;

    class ImageProjection {
    public:

        pcl::PointCloud<PointType>::Ptr laserCloudIn;
        pcl::PointCloud<PointType>::Ptr laserCloudIn_last_;

        pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
        pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range
        pcl::PointCloud<PointType>::Ptr LinearPoints_;
        pcl::PointCloud<PointType>::Ptr Less_LinearPoints_;
        pcl::PointCloud<PointType>::Ptr Less_LinearPoints_D_;
        pcl::PointCloud<PointType>::Ptr LinearPoints_last_;
        pcl::PointCloud<PointType>::Ptr fullLinearPoints_;

        pcl::PointCloud<PointType>::Ptr fullValidPoints_z_;
        pcl::PointCloud<PointType>::Ptr fullValidPoints_smoothness_;

        pcl::PointCloud<PointType>::Ptr LinearPoints_Ground;
        pcl::PointCloud<PointType>::Ptr LinearPoints_Less_Ground;


        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        float avg_smoothness_;
        float avg_smooth_predict_;
        float sigma_smooth_predict_;
        vector<float> predicted_sigma_set_;

        float avg_smooth_predict_vs_;
        float sigma_smooth_predict_vs_;

        float traget_distribution(float smooth, double s);

        vector <PointType> neighbor_set_;

        pcl::PointCloud<PointType>::Ptr groundCloud;
        pcl::PointCloud<PointType>::Ptr segmentedCloud;
        pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
        pcl::PointCloud<PointType>::Ptr outlierCloud;

        pcl::KdTreeFLANN<PointType>::Ptr kdtree_linear_;

        bool use_prob_smooth_;

        int feature_no_;

        PointType nanPoint; // fill in fullCloud at each iteration

        cv::Mat rangeMat; // range matrix for range image
        cv::Mat labelMat; // label matrix for segmentaiton marking
        cv::Mat groundMat; // ground matrix for ground cloud marking
        int labelCount;

        float startOrientation;
        float endOrientation;

        Vector21f last_distr_;

        cloud_info segMsg; // info of segmented cloud

        std::vector <std::pair<int8_t, int8_t>> neighborIterator; // neighbor iterator for segmentaiton process

        uint16_t *allPushedIndX; // array for tracking points of a segmented object
        uint16_t *allPushedIndY;

        uint16_t *queueIndX; // array for breadth-first search process of segmentation
        uint16_t *queueIndY;

        Vec16f smooth_thre_;

        float plane_no_;

        double Time_;
        double Thre_;

        pcl::KdTreeFLANN<PointType>::Ptr kdtree_;

        ImageProjection();

        Eigen::Matrix4d Init_Trans_;

        void allocateMemory();

        void resetParameters();

        void cloudHandler(pcl::PointCloud<PointType>::Ptr input_cloud);

        void findStartEndAngle();

        void projectPointCloud();

        void groundRemoval();

        void extractLinearPoints();

        void showTwoPC(pcl::PointCloud<PointType>::Ptr curr_cloud_gicp, pcl::PointCloud<PointType>::Ptr ref_cloud);
    };
}
#endif //LOAM_ODOMETRY_IMAGEPROJECTION_H
