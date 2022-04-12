//
// Created by robot410 on 2020/11/3.
//

#ifndef LOAM_ODOMETRY_FEATUREASSOCIATION_H
#define LOAM_ODOMETRY_FEATUREASSOCIATION_H

#include "hdl_graph_slam/utility.h"
typedef Eigen::Matrix<float, 21, 1> Vector21f;
namespace hdl_graph_slam {
    class FeatureAssociation {

    public:

        pcl::PointCloud<PointType>::Ptr segmentedCloud;
        pcl::PointCloud<PointType>::Ptr outlierCloud;
        pcl::PointCloud <PointType> LaserCloud_;
        pcl::PointCloud <PointType> LaserCloud_last_;

        pcl::PointCloud<PointType> LinearCloud_;
        pcl::PointCloud <PointType> LinearCloud_last_;

        pcl::PointCloud<PointType>::Ptr Source_filtered_;
        pcl::PointCloud<PointType>::Ptr Target_filtered_;

        pcl::PointCloud<PointType> Less_LinearCloud_;
        pcl::PointCloud <PointType> Less_LinearCloud_last_;

        pcl::PointCloud<PointType> LinearCloud_Ground_;
        pcl::PointCloud<PointType> LinearCloud_Less_Ground_;

        pcl::PointCloud <PointType> LinearCloud_Ground_Last;
        pcl::PointCloud <PointType> LinearCloud_Less_Ground_Last;

        pcl::PointCloud<PointType>::Ptr cornerPointsSharp;
        pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp;
        pcl::PointCloud<PointType>::Ptr surfPointsFlat;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlat;

        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan;
        pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScanDS;

        pcl::VoxelGrid <PointType> downSizeFilter;

        Eigen::Matrix4d RL_transfomation_;

        Eigen::MatrixXd Pose_COV_;

        double timeScanCur;
        double timeNewSegmentedCloud;
        double timeNewSegmentedCloudInfo;
        double timeNewOutlierCloud;

        bool newSegmentedCloud;
        bool newSegmentedCloudInfo;
        bool newOutlierCloud;
        double Time_;
        double Thre_;
        bool use_sampling_thre_;

        cloud_info segInfo;

        int systemInitCount;
        bool systemInited;

        std::vector <smoothness_t> cloudSmoothness;
        float cloudCurvature[N_SCAN * Horizon_SCAN];
        int cloudNeighborPicked[N_SCAN * Horizon_SCAN];
        int cloudLabel[N_SCAN * Horizon_SCAN];

        int imuPointerFront;
        int imuPointerLast;
        int imuPointerLastIteration;

        float imuRollStart, imuPitchStart, imuYawStart;
        float cosImuRollStart, cosImuPitchStart, cosImuYawStart, sinImuRollStart, sinImuPitchStart, sinImuYawStart;
        float imuRollCur, imuPitchCur, imuYawCur;

        float imuVeloXStart, imuVeloYStart, imuVeloZStart;
        float imuShiftXStart, imuShiftYStart, imuShiftZStart;

        float imuVeloXCur, imuVeloYCur, imuVeloZCur;
        float imuShiftXCur, imuShiftYCur, imuShiftZCur;

        float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
        float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

        float imuAngularRotationXCur, imuAngularRotationYCur, imuAngularRotationZCur;
        float imuAngularRotationXLast, imuAngularRotationYLast, imuAngularRotationZLast;
        float imuAngularFromStartX, imuAngularFromStartY, imuAngularFromStartZ;

        double imuTime[imuQueLength];
        float imuRoll[imuQueLength];
        float imuPitch[imuQueLength];
        float imuYaw[imuQueLength];

        float imuAccX[imuQueLength];
        float imuAccY[imuQueLength];
        float imuAccZ[imuQueLength];

        float imuVeloX[imuQueLength];
        float imuVeloY[imuQueLength];
        float imuVeloZ[imuQueLength];

        float imuShiftX[imuQueLength];
        float imuShiftY[imuQueLength];
        float imuShiftZ[imuQueLength];

        float imuAngularVeloX[imuQueLength];
        float imuAngularVeloY[imuQueLength];
        float imuAngularVeloZ[imuQueLength];

        float imuAngularRotationX[imuQueLength];
        float imuAngularRotationY[imuQueLength];
        float imuAngularRotationZ[imuQueLength];

        int skipFrameNum;
        bool systemInitedLM;

        int laserCloudCornerLastNum;
        int laserCloudSurfLastNum;

        int pointSelCornerInd[N_SCAN * Horizon_SCAN];
        float pointSearchCornerInd1[N_SCAN * Horizon_SCAN];
        float pointSearchCornerInd2[N_SCAN * Horizon_SCAN];

        int pointSelSurfInd[N_SCAN * Horizon_SCAN];
        float pointSearchSurfInd1[N_SCAN * Horizon_SCAN];
        float pointSearchSurfInd2[N_SCAN * Horizon_SCAN];
        float pointSearchSurfInd3[N_SCAN * Horizon_SCAN];

        float transformCur[6];
        float transformCur_GT[6];
        float transformSum[6];
        float transformIncre[0];

        float mean_matches_errors_;
        float matched_f_no_;
        float total_f_no_;

        float Loam_mean_matches_errors_;
        float Loam_matched_f_no_;
        float Loam_total_f_no_;


        float imuRollLast, imuPitchLast, imuYawLast;
        float imuShiftFromStartX, imuShiftFromStartY, imuShiftFromStartZ;
        float imuVeloFromStartX, imuVeloFromStartY, imuVeloFromStartZ;

        pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
        pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
        pcl::PointCloud<PointType>::Ptr laserCloudOri;
        pcl::PointCloud<PointType>::Ptr laserCloudOri_sub;
        pcl::PointCloud<PointType>::Ptr coeffSel;
        pcl::PointCloud<PointType>::Ptr target_cloud_;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeLineCornerLast;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGroundCornerLast;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast;
        pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast;

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        vector<float> init_error_;
        float a1, a2, a3, a4, a5, a6, a7;
        int n1, n2, n3, n4, n5, n6, n7;
        vector<float> v1, v2, v3, v4, v5, v6, v7;
        vector<float> sigma_set_;
        double error_sum;
        float avg_smooth_;
        float sigma_smooth_;
        float sigma_smooth_vs_;
        int avg_smooth_no_;
        int segmentedCloudNum_;
        float avg_error_last_;
        Vector21f smooth_dist, smooth_dist_no_;
        vector<float> truth_smooth_set_;
        vector<float> false_smooth_set_;
        float avg_smooth_vs_;
        int avg_smooth_no_vs_;


        float max_prob_;

        double total_cost_ = 0;
        Eigen::Matrix4d GT_;

        PointType pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff;

        bool isDegenerate;
        cv::Mat matP;

        int iterCount1_, iterCount2_;
        int frameCount;

        FeatureAssociation();

        void TransformToStart(PointType const *const pi, PointType *const po);

        double rad2deg(double radians);

        //double deg2rad(double degrees);

        void findCorrespondingCornerFeatures_X(int iterCount);

        void initializationValue();

        void updateTransformation();

        void findCorrespondingSurfFeatures(int iterCount);

        bool calculateTransformationSurf(int iterCount);

        bool calculateTransformationCorner(int iterCount);

        bool calculateTransformation(int iterCount);

        void runFeatureAssociation(Eigen::Matrix4f Init_state);

        void Matrix4dToRpyxyz(const Eigen::Matrix4f &Trans,float nums[]);

        void toEulerAngle( Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw);

        void setSourceCloud(const pcl::PointCloud<PointType>::ConstPtr cloud);

        void setTargetCloud(const pcl::PointCloud<PointType>::ConstPtr cloud);
        void setInitTargetCloud(const pcl::PointCloud<PointType>::ConstPtr cloud);


        Eigen::Matrix4f getFinalTransformation();

        void RpyxyzToMatrix4f(Eigen::Matrix4f &Trans, float nums[]);
    };
}
#endif //LOAM_ODOMETRY_FEATUREASSOCIATION_H
