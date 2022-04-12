#include "hdl_graph_slam/imageProjection.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

using namespace hdl_graph_slam;
ImageProjection::ImageProjection() {

    nanPoint.x = std::numeric_limits<float>::quiet_NaN();
    nanPoint.y = std::numeric_limits<float>::quiet_NaN();
    nanPoint.z = std::numeric_limits<float>::quiet_NaN();
    nanPoint.intensity = -1;

    allocateMemory();
    resetParameters();
}

void ImageProjection::allocateMemory() {
    if(Time_<10)
        last_distr_.setOnes();
    sigma_smooth_predict_ = 6;
    laserCloudIn.reset(new pcl::PointCloud<PointType>());
    laserCloudIn_last_.reset(new pcl::PointCloud<PointType>());
    LinearPoints_.reset(new pcl::PointCloud<PointType>());
    LinearPoints_last_.reset(new pcl::PointCloud<PointType>());
    fullLinearPoints_.reset(new pcl::PointCloud<PointType>());
    LinearPoints_Ground.reset(new pcl::PointCloud<PointType>());
    LinearPoints_Less_Ground.reset(new pcl::PointCloud<PointType>());
    Less_LinearPoints_.reset(new pcl::PointCloud<PointType>());
    Less_LinearPoints_D_.reset(new pcl::PointCloud<PointType>());


    fullValidPoints_z_.reset(new pcl::PointCloud<PointType>());
    fullValidPoints_smoothness_.reset(new pcl::PointCloud<PointType>());
   
    fullCloud.reset(new pcl::PointCloud<PointType>());
    fullInfoCloud.reset(new pcl::PointCloud<PointType>());

    groundCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    fullCloud->points.resize(N_SCAN * Horizon_SCAN);
    fullInfoCloud->points.resize(N_SCAN * Horizon_SCAN);

    segMsg.startRingIndex = new int[N_SCAN];
    segMsg.endRingIndex = new int[N_SCAN];

    segMsg.segmentedCloudGroundFlag = new bool[N_SCAN * Horizon_SCAN];
    segMsg.segmentedCloudColInd = new int[N_SCAN * Horizon_SCAN];
    segMsg.segmentedCloudRange = new float[N_SCAN * Horizon_SCAN];

    for (int i = 0; i < N_SCAN * Horizon_SCAN; i++) {
        segMsg.segmentedCloudGroundFlag[i] = false;
        segMsg.segmentedCloudColInd[i] = 0;
        segMsg.segmentedCloudRange[i] = 0;
    }

    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = 1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = -1;
    neighborIterator.push_back(neighbor);
    neighbor.first = 1;
    neighbor.second = 0;
    neighborIterator.push_back(neighbor);

    allPushedIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    allPushedIndY = new uint16_t[N_SCAN * Horizon_SCAN];

    queueIndX = new uint16_t[N_SCAN * Horizon_SCAN];
    queueIndY = new uint16_t[N_SCAN * Horizon_SCAN];


}

void ImageProjection::resetParameters() {
    laserCloudIn->clear();
    //laserCloudIn_last_->clear();
    groundCloud->clear();
    segmentedCloud->clear();
    segmentedCloudPure->clear();
    outlierCloud->clear();
    LinearPoints_->clear();
    Less_LinearPoints_->clear();
    Less_LinearPoints_D_->clear();
    fullLinearPoints_->clear();
    LinearPoints_Ground->clear();
    LinearPoints_Less_Ground->clear();

    fullValidPoints_z_->clear();
    fullValidPoints_smoothness_->clear();

    rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
    groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
    labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
    labelCount = 1;

    std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
    std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
}

void ImageProjection::showTwoPC(pcl::PointCloud<PointType>::Ptr curr_cloud_gicp, pcl::PointCloud<PointType>::Ptr ref_cloud) {

    bool visualize = true;

    if (visualize) {
        int v1(0);
        using pcl::visualization::PCLVisualizer;
        using pcl::visualization::PointCloudColorHandlerGenericField;
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::PointCloud<PointType>::Ptr curr_cloud_ptr(curr_cloud_gicp);//icp_output
        pcl::PointCloud<PointType>::Ptr ref_cloud_ptr(ref_cloud);
        PointCloudColorHandlerGenericField<PointType> fildColor(curr_cloud_ptr, "z");
        pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color0(curr_cloud_ptr, 0, 255, 0); // green

        viewer->addPointCloud<PointType>(curr_cloud_ptr, single_color0, "sample cloud");
        pcl::visualization::PointCloudColorHandlerCustom<PointType> single_color1(ref_cloud_ptr, 255, 0, 0); // red
        viewer->addPointCloud<PointType>(ref_cloud_ptr, single_color1, "sample cloud1");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        //viewer->createViewPort(0,0,1,1,v1);
        viewer->addCoordinateSystem(1.0);
        viewer->initCameraParameters();

        while (!viewer->wasStopped()) {
            viewer->spinOnce(100);
            //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
    }
}


void ImageProjection::cloudHandler(pcl::PointCloud<PointType>::Ptr input_cloud) {

    clock_t start0 = clock();
    for (int i = 0; i < input_cloud->size(); i++)
        laserCloudIn->push_back(input_cloud->points[i]);

    clock_t start = clock();
    //std::cout<<"get pc"<<(double)(start-start0)/CLOCKS_PER_SEC<<std::endl;
    findStartEndAngle();
    clock_t end0 = clock();
    //std::cout<<"findStartEndAngle"<<(double)(end0-start)/CLOCKS_PER_SEC<<std::endl;
    projectPointCloud();
    clock_t end1 = clock();
    //std::cout<<"projectPointCloud"<<(double)(end1-end0)/CLOCKS_PER_SEC<<std::endl;
    groundRemoval();
    extractLinearPoints();
    clock_t end2 = clock();
    //std::cout<<"extractLinearPoints"<<(double)(end2-end1)/CLOCKS_PER_SEC<<std::endl;
}

void ImageProjection::findStartEndAngle() {
    // start and end orientation of this cloud
    segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
    segMsg.endOrientation = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                   laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;
    if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
        segMsg.endOrientation -= 2 * M_PI;
    } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
        segMsg.endOrientation += 2 * M_PI;
    segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
}

void ImageProjection::projectPointCloud() {
    // range image projection
    float verticalAngle, horizonAngle, range;
    size_t rowIdn, columnIdn, index, cloudSize;
    PointType thisPoint;

    cloudSize = laserCloudIn->points.size();

    for (size_t i = 0; i < cloudSize; ++i) {

        thisPoint.x = laserCloudIn->points[i].x;
        thisPoint.y = laserCloudIn->points[i].y;
        thisPoint.z = laserCloudIn->points[i].z;
        // find the row and column index in the image for this point
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
        if (rowIdn < 0 || rowIdn >= N_SCAN)
            continue;

        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;

        columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;
        if (columnIdn >= Horizon_SCAN)
            columnIdn -= Horizon_SCAN;

        if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
        if (range < 0.5)
            continue;

        rangeMat.at<float>(rowIdn, columnIdn) = range;

        thisPoint.intensity = (float) rowIdn + (float) columnIdn / 10000.0;

        index = columnIdn + rowIdn * Horizon_SCAN;
        fullCloud->points[index] = thisPoint;
        fullInfoCloud->points[index] = thisPoint;
        fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
    }

}


void ImageProjection::extractLinearPoints() {

    use_prob_smooth_ = true;
    size_t upperInd, currInd, leftInd_1, rightInd_1, leftInd_2, rightInd_2, leftInd_3,
            rightInd_3, lowInd,rightupperInd_1,leftupperInd_1;
    int point_no = 0,  point_no_g(0), point_no_less_g(0), point_no_less = 0,point_no_full(0);

    plane_no_ = 0;


    for (size_t j = 3; j < Horizon_SCAN - 4; j = j+1) {
        for (size_t i = 1; i < N_SCAN - 1; ++i) {
            lowInd = j + (i - 1) * Horizon_SCAN;
            currInd = j + i * Horizon_SCAN;
            upperInd = j + (i + 1) * Horizon_SCAN;
            leftInd_1 = j - 1 + i * Horizon_SCAN;
	        leftupperInd_1 = j - 1 + (i+1) * Horizon_SCAN;
            rightInd_1 = j + 1 + i * Horizon_SCAN;
	        rightupperInd_1 = j + 1 + (i+1) * Horizon_SCAN;
            leftInd_2 = j - 2 + i * Horizon_SCAN;
            rightInd_2 = j + 2 + i * Horizon_SCAN;
            leftInd_3 = j - 3 + i * Horizon_SCAN;
            rightInd_3 = j + 3 + i * Horizon_SCAN;

            if (fullInfoCloud->points[currInd].intensity == -1 ||
                fullInfoCloud->points[upperInd].intensity == -1 ||
                fullInfoCloud->points[leftInd_1].intensity == -1 ||
                fullInfoCloud->points[rightInd_1].intensity == -1 
	       ) {
                continue;
            }

            double smoothness =
                    fullInfoCloud->points[leftInd_1].intensity + fullInfoCloud->points[leftInd_2].intensity +
                    fullInfoCloud->points[leftInd_3].intensity - 6 * fullInfoCloud->points[currInd].intensity +
                    fullInfoCloud->points[rightInd_1].intensity + fullInfoCloud->points[rightInd_2].intensity +
                    fullInfoCloud->points[rightInd_3].intensity;
            PointType upper_normals0;
            upper_normals0.x = fullInfoCloud->points[upperInd].x - fullInfoCloud->points[currInd].x;
            upper_normals0.y = fullInfoCloud->points[upperInd].y - fullInfoCloud->points[currInd].y;
            upper_normals0.z = fullInfoCloud->points[upperInd].z - fullInfoCloud->points[currInd].z;
            upper_normals0.intensity = 0;

            float norm = sqrt(
                    upper_normals0.x * upper_normals0.x + upper_normals0.y * upper_normals0.y + upper_normals0.z *
                                                                                                upper_normals0.z);
            upper_normals0.z = upper_normals0.z / norm;
            smoothness = abs(smoothness) / 6;


            float z_thre = 0.9;
            if (abs(upper_normals0.z) > z_thre and groundMat.at<int8_t>(i, j) > -1) //
            {
                PointType p = fullInfoCloud->points[currInd];
                p.intensity = smoothness;
                Less_LinearPoints_->points.push_back(p);
                point_no_less++;
                p.intensity = smoothness;
                if(smoothness > 1 and currInd%2 == 0) //and currInd%2 == 0
                {
                    LinearPoints_->points.push_back(p);
                    ++point_no;
                }
            }

            if(abs(upper_normals0.z)<0.3 ) // and groundMat.at<int8_t>(i, j) == 1
            {
                if (currInd % 2 == 0) {
                    LinearPoints_Less_Ground->points.push_back(fullInfoCloud->points[currInd]);
                    ++point_no_less_g;
                }
                if (currInd % 4 == 0) {
                    LinearPoints_Ground->points.push_back(fullInfoCloud->points[currInd]);
                    ++point_no_g;
                }
            }
        }
    }
/*
    for (size_t j = 0; j < Horizon_SCAN; ++j) {
        for (size_t i = 0; i < groundScanInd; ++i) {
            size_t currInd = j + (i) * Horizon_SCAN;
            if(groundMat.at<int8_t>(i, j) == 1) {
                if (currInd % 4 == 0) {
                    LinearPoints_Less_Ground->points.push_back(fullInfoCloud->points[currInd]);
                    ++point_no_less_g;
                }
                if (currInd % 16 == 0) {
                    LinearPoints_Ground->points.push_back(fullInfoCloud->points[currInd]);
                    ++point_no_g;
                }
            }
        }
    }*/

    //smooth_f.close();
    LinearPoints_->width = point_no;
    LinearPoints_->height = 1;
    LinearPoints_->is_dense = false;
    LinearPoints_->points.resize(LinearPoints_->width * LinearPoints_->height);
   // cout<<"LinearPoints_->size()"<<" "<<LinearPoints_->size()<<" "<<laserCloudIn->size()<<endl;

    Less_LinearPoints_->width = point_no_less;
    Less_LinearPoints_->height = 1;
    Less_LinearPoints_->is_dense = false;
    Less_LinearPoints_->points.resize(Less_LinearPoints_->width * Less_LinearPoints_->height);
    //cout<<"Less_LinearPoints_->size()"<<" "<<Less_LinearPoints_->size()<<endl;


    LinearPoints_Ground->width = point_no_g;
    LinearPoints_Ground->height = 1;
    LinearPoints_Ground->is_dense = false;
    LinearPoints_Ground->points.resize(LinearPoints_Ground->width * LinearPoints_Ground->height);
   // cout<<"LinearPoints_Ground->size()"<<" "<<LinearPoints_Ground->size()<<endl;

    LinearPoints_Less_Ground->width = point_no_less_g;
    LinearPoints_Less_Ground->height = 1;
    LinearPoints_Less_Ground->is_dense = false;
    LinearPoints_Less_Ground->points.resize(LinearPoints_Less_Ground->width * LinearPoints_Less_Ground->height);
    //cout<<"LinearPoints_Less_Ground->size()"<<" "<<LinearPoints_Less_Ground->size()<<endl;

}




void ImageProjection::groundRemoval() {
    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    // groundMat
    // -1, no valid info to check if ground of not
    //  0, initial value, after validation, means not ground
    //  1, ground
    for (size_t j = 0; j < Horizon_SCAN; ++j) {
        for (size_t i = 0; i < groundScanInd; ++i) {

            lowerInd = j + (i) * Horizon_SCAN;
            upperInd = j + (i + 1) * Horizon_SCAN;

            if (fullCloud->points[lowerInd].intensity == -1 ||
                fullCloud->points[upperInd].intensity == -1) {
                // no info to check, invalid points
                groundMat.at<int8_t>(i, j) = -1;
                continue;
            }

            diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
            diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
            diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;

            angle = atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

            if (abs(angle - sensorMountAngle) <= 10) {
                groundMat.at<int8_t>(i, j) = 1;
                groundMat.at<int8_t>(i + 1, j) = 1;

            }
        }
    }
    // extract ground cloud (groundMat == 1)
    // mark entry that doesn't need to label (ground and invalid point) for segmentation
    // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
    for (size_t i = 0; i < N_SCAN; ++i) {
        for (size_t j = 0; j < Horizon_SCAN; ++j) {
            if (groundMat.at<int8_t>(i, j) == 1 || rangeMat.at<float>(i, j) == FLT_MAX) {
                labelMat.at<int>(i, j) = -1;
            }
        }
    }
}
