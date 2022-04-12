#include "hdl_graph_slam/featureAssociation.h"
#include "hdl_graph_slam/imageProjection.h"
#include <iostream>
#include <cmath>

using namespace hdl_graph_slam;

FeatureAssociation::FeatureAssociation() {

    initializationValue();
}

void FeatureAssociation::initializationValue() {
    max_prob_ = 0.0;
    avg_error_last_ = 0.2;
    cloudSmoothness.resize(N_SCAN * Horizon_SCAN);

    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

    segmentedCloud.reset(new pcl::PointCloud<PointType>());
    outlierCloud.reset(new pcl::PointCloud<PointType>());

    Source_filtered_.reset(new pcl::PointCloud<PointType>());
    Target_filtered_.reset(new pcl::PointCloud<PointType>());

    LinearCloud_.makeShared().reset(new pcl::PointCloud<PointType>());
    Less_LinearCloud_.makeShared().reset(new pcl::PointCloud<PointType>());
    LinearCloud_Ground_.makeShared().reset(new pcl::PointCloud<PointType>());
    LinearCloud_Less_Ground_.makeShared().reset(new pcl::PointCloud<PointType>());

    Less_LinearCloud_last_.makeShared().reset(new pcl::PointCloud<PointType>());
    LinearCloud_Less_Ground_Last.makeShared().reset(new pcl::PointCloud<PointType>());

    cornerPointsSharp.reset(new pcl::PointCloud<PointType>());
    cornerPointsLessSharp.reset(new pcl::PointCloud<PointType>());
    surfPointsFlat.reset(new pcl::PointCloud<PointType>());
    surfPointsLessFlat.reset(new pcl::PointCloud<PointType>());

    surfPointsLessFlatScan.reset(new pcl::PointCloud<PointType>());
    surfPointsLessFlatScanDS.reset(new pcl::PointCloud<PointType>());

    timeScanCur = 0;
    timeNewSegmentedCloud = 0;
    timeNewSegmentedCloudInfo = 0;
    timeNewOutlierCloud = 0;

    newSegmentedCloud = false;
    newSegmentedCloudInfo = false;
    newOutlierCloud = false;

    systemInitCount = 0;
    systemInited = false;

    for (int i = 0; i < 6; ++i) {
        transformCur[i] = 0;
        transformSum[i] = 0;
    }

    systemInitedLM = false;

    laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
    laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
    laserCloudOri.reset(new pcl::PointCloud<PointType>());
    coeffSel.reset(new pcl::PointCloud<PointType>());
    target_cloud_.reset(new pcl::PointCloud<PointType>());

    kdtreeLineCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeCornerLast.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeSurfLast.reset(new pcl::KdTreeFLANN<PointType>());
    kdtreeGroundCornerLast.reset(new pcl::KdTreeFLANN<PointType>());

    isDegenerate = false;
    matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

    frameCount = 1;

    smooth_dist.setZero();
    smooth_dist_no_.setZero();
}

void FeatureAssociation::setSourceCloud(const pcl::PointCloud<PointType>::ConstPtr cloud)
{
    pcl::copyPointCloud(*cloud,LaserCloud_);

    ImageProjection IP_source;
    IP_source.Thre_ = Thre_;
    IP_source.Time_ = Time_;
    clock_t s1 = clock();
    IP_source.cloudHandler(LaserCloud_.makeShared());
    clock_t e1 = clock();
    pcl::copyPointCloud(*IP_source.LinearPoints_,LinearCloud_);
    pcl::copyPointCloud(*IP_source.Less_LinearPoints_,Less_LinearCloud_);
    pcl::copyPointCloud(*IP_source.LinearPoints_Less_Ground,LinearCloud_Less_Ground_);
    pcl::copyPointCloud(*IP_source.LinearPoints_Ground,LinearCloud_Ground_);
    clock_t e2 = clock();

    cout<<"setSourceCloud "<<(double)(e2-e1)/CLOCKS_PER_SEC<<" "<<(double)(e1-s1)/CLOCKS_PER_SEC<<endl;
}
void FeatureAssociation::setTargetCloud(const pcl::PointCloud<PointType>::ConstPtr cloud)
{

    pcl::copyPointCloud(*cloud,LaserCloud_last_);
    /*
    ImageProjection IP_target;
    IP_target.Thre_ = Thre_;
    IP_target.Time_ = Time_;
    IP_target.cloudHandler(LaserCloud_last_.makeShared());*/
    pcl::copyPointCloud(LinearCloud_,LinearCloud_last_);
    pcl::copyPointCloud(Less_LinearCloud_,Less_LinearCloud_last_);
    pcl::copyPointCloud(LinearCloud_Less_Ground_,LinearCloud_Less_Ground_Last);
    pcl::copyPointCloud(LinearCloud_Ground_,LinearCloud_Ground_Last);
    cout<<"setTargetCloud"<<endl;
}

void FeatureAssociation::setInitTargetCloud(const pcl::PointCloud<PointType>::ConstPtr cloud)
{

    pcl::copyPointCloud(*cloud,LaserCloud_last_);

    ImageProjection IP_target;
    IP_target.Thre_ = Thre_;
    IP_target.Time_ = Time_;
    IP_target.cloudHandler(LaserCloud_last_.makeShared());
    pcl::copyPointCloud(*IP_target.LinearPoints_,LinearCloud_last_);
    pcl::copyPointCloud(*IP_target.Less_LinearPoints_,Less_LinearCloud_last_);
    pcl::copyPointCloud(*IP_target.LinearPoints_Less_Ground,LinearCloud_Less_Ground_Last);
    pcl::copyPointCloud(*IP_target.LinearPoints_Ground,LinearCloud_Ground_Last);
    cout<<"setTargetCloud"<<endl;
}

void FeatureAssociation::TransformToStart(PointType const *const pi, PointType *const po) {
    float s = 1; //10 * (pi->intensity - int(pi->intensity));

    float rx = s * transformCur[0];
    float ry = s * transformCur[1];
    float rz = s * transformCur[2];
    float tx = s * transformCur[3];
    float ty = s * transformCur[4];
    float tz = s * transformCur[5];

    float x1 = cos(rz) * (pi->x) - sin(rz) * (pi->y);
    float y1 = sin(rz) * (pi->x) + cos(rz) * (pi->y);
    float z1 = (pi->z);

    float x2 = cos(ry) * x1 + sin(ry) * z1;
    float y2 = y1;
    float z2 = -sin(ry) * x1 + cos(ry) * z1;

    po->x = x2 + tx;
    po->y = cos(rx) * y2 - sin(rx) * z2 + ty;
    po->z = sin(rx) * y2 + cos(rx) * z2 + tz;
    po->intensity = pi->intensity;
}

double FeatureAssociation::rad2deg(double radians) {
    return radians * 180.0 / M_PI;
}

void FeatureAssociation::findCorrespondingCornerFeatures_X(int iterCount) {
    smooth_dist.setOnes();
    smooth_dist_no_.setZero();
    total_cost_ = 0;
    //std::cout<<"linear points: "<<Less_LinearCloud_last_.size()<<std::endl;
    kdtreeLineCornerLast->setInputCloud(Less_LinearCloud_last_.makeShared());
    int segmentedCloudNum = LinearCloud_.points.size();
    //std::cout<<"linear points: "<<segmentedCloudNum<<" "<<Less_LinearCloud_last_.size()<<std::endl;
    for (int i = 0; i < segmentedCloudNum; i++) {
        //then, for each sharp corner point, transform the current point to the reference coordinates,termed as pointSel
        TransformToStart(&LinearCloud_.points[i], &pointSel);

        if (iterCount % 5 == 0) {
            //During the checkSystemInitialization() process, have set the PointCloud in last frame, kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
            PointType p0 = pointSel;
            //p0.z = 0;
            kdtreeLineCornerLast->nearestKSearch(p0, 1, pointSearchInd, pointSearchSqDis);
            int closestPointInd = -1, minPointInd2 = -1;
            // the two indexs
            if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                closestPointInd = pointSearchInd[0];
            }

            pointSearchCornerInd1[i] = closestPointInd;
        }
        //compute the point-to-line distance according to the cosine theory and the deviretion
        if (pointSearchCornerInd1[i] >= 0) {

            tripod1 = Less_LinearCloud_last_.points[pointSearchCornerInd1[i]];
            //tripod2 = Less_LinearCloud_last_D_.points[pointSearchCornerInd1[i]];

            float x0 = pointSel.x;
            float y0 = pointSel.y;
            float z0 = pointSel.z;
            float x1 = tripod1.x;
            float y1 = tripod1.y;
            float z1 = tripod1.z;
            float x2 = tripod1.x;
            float y2 = tripod1.y;
            float z2 = tripod1.z + 0.5;

            float m11 = ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1));
            float m22 = ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1));
            float m33 = ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1));

            float a012 = sqrt(m11 * m11 + m22 * m22 + m33 * m33);

            float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

            float la = ((y1 - y2) * m11 + (z1 - z2) * m22) / a012 / l12;

            float lb = -((x1 - x2) * m11 - (z1 - z2) * m33) / a012 / l12;

            float lc = -((x1 - x2) * m22 + (y1 - y2) * m33) / a012 / l12;

            float ld2 = a012 / l12;

            float s = 1;
            if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(ld2);
            }


            error_sum += abs(ld2);

            if (s > 0.1) {
                coeff.x = s * la;
                coeff.y =  s * lb;
                coeff.z =  s * lc;
                coeff.intensity =  s * ld2;
                LinearCloud_.points[i].intensity = ld2;
                laserCloudOri->push_back(LinearCloud_.points[i]);
                coeffSel->push_back(coeff);
                //
                target_cloud_->push_back(tripod1);
            }
        }
    }
}




void FeatureAssociation::findCorrespondingSurfFeatures(int iterCount) {
    //cout<<"LinearCloud_Less_Ground_Last "<<LinearCloud_Less_Ground_Last.points.size()<<endl;
    kdtreeGroundCornerLast->setInputCloud(LinearCloud_Less_Ground_Last.makeShared());
    total_cost_ = 0;
    int surfPointsFlatNum = LinearCloud_Ground_.points.size();

    for (int i = 0; i < surfPointsFlatNum; i++) {
        //cout<<"begin transformto start"<<endl;
        TransformToStart(&LinearCloud_Ground_.points[i], &pointSel);
        //cout<<"finish transformto start "<<pointSel.x<<" "<<LinearCloud_Ground_.points[i].x<<endl;
        if (iterCount % 5 == 0) {

            kdtreeGroundCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;

            if (pointSearchSqDis[0] < nearestFeatureSearchSqDist) {
                closestPointInd = pointSearchInd[0];
                int closestPointScan = int(LinearCloud_Less_Ground_Last.points[closestPointInd].intensity);

                float pointSqDis, minPointSqDis2 = nearestFeatureSearchSqDist, minPointSqDis3 = nearestFeatureSearchSqDist;
                for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                    if (int(LinearCloud_Less_Ground_Last.points[j].intensity) > closestPointScan + 2.5) {
                        break;
                    }

                    pointSqDis = (LinearCloud_Less_Ground_Last.points[j].x - pointSel.x) *
                                 (LinearCloud_Less_Ground_Last.points[j].x - pointSel.x) +
                                 (LinearCloud_Less_Ground_Last.points[j].y - pointSel.y) *
                                 (LinearCloud_Less_Ground_Last.points[j].y - pointSel.y) +
                                 (LinearCloud_Less_Ground_Last.points[j].z - pointSel.z) *
                                 (LinearCloud_Less_Ground_Last.points[j].z - pointSel.z);

                    if (int(LinearCloud_Less_Ground_Last.points[j].intensity) <= closestPointScan) {
                        if (pointSqDis < minPointSqDis2) {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    } else {
                        if (pointSqDis < minPointSqDis3) {
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }
                }
                for (int j = closestPointInd - 1; j >= 0; j--) {
                    if (int(LinearCloud_Less_Ground_Last.points[j].intensity) < closestPointScan - 2.5) {
                        break;
                    }

                    pointSqDis = (LinearCloud_Less_Ground_Last.points[j].x - pointSel.x) *
                                 (LinearCloud_Less_Ground_Last.points[j].x - pointSel.x) +
                                 (LinearCloud_Less_Ground_Last.points[j].y - pointSel.y) *
                                 (LinearCloud_Less_Ground_Last.points[j].y - pointSel.y) +
                                 (LinearCloud_Less_Ground_Last.points[j].z - pointSel.z) *
                                 (LinearCloud_Less_Ground_Last.points[j].z - pointSel.z);

                    if (int(LinearCloud_Less_Ground_Last.points[j].intensity) >= closestPointScan) {
                        if (pointSqDis < minPointSqDis2) {
                            minPointSqDis2 = pointSqDis;
                            minPointInd2 = j;
                        }
                    } else {
                        if (pointSqDis < minPointSqDis3) {
                            minPointSqDis3 = pointSqDis;
                            minPointInd3 = j;
                        }
                    }
                }
            }

            pointSearchSurfInd1[i] = closestPointInd;
            pointSearchSurfInd2[i] = minPointInd2;
            pointSearchSurfInd3[i] = minPointInd3;
        }

        if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {

            tripod1 = LinearCloud_Less_Ground_Last.points[pointSearchSurfInd1[i]];
            tripod2 = LinearCloud_Less_Ground_Last.points[pointSearchSurfInd2[i]];
            tripod3 = LinearCloud_Less_Ground_Last.points[pointSearchSurfInd3[i]];

            tripod2.z = tripod1.z;
            tripod3.z = tripod1.z;

            float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z)
                       - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
            float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x)
                       - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
            float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y)
                       - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
            float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

            float ps = sqrt(pa * pa + pb * pb + pc * pc);

            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

            float s = 1;
            if (iterCount >= 5) {
                s = 1 - 1.8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                                                    + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
            }

            if (s > 0.1 && abs(pd2)>0) {
                coeff.x = s * pa;
                coeff.y = s * pb;
                coeff.z = s * pc;
                coeff.intensity = s * pd2;
                total_cost_ = total_cost_ + pd2;
                laserCloudOri->push_back(LinearCloud_Ground_.points[i]);
                //tripod1.x = pointSel.x;
                //tripod1.y = pointSel.y;
                target_cloud_->push_back(tripod1);
                coeffSel->push_back(coeff);
            }
        }
    }
}

bool FeatureAssociation::calculateTransformationSurf(int iterCount) {

    int pointSelNum = laserCloudOri->points.size();
    //cout<<"Ground pointSelNum: "<<pointSelNum<<endl;
    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    float srx = sin(transformCur[0]);
    float crx = cos(transformCur[0]);
    float sry = sin(transformCur[1]);
    float cry = cos(transformCur[1]);
    float srz = sin(transformCur[2]);
    float crz = cos(transformCur[2]);
    float tx = transformCur[3];
    float ty = transformCur[4];
    float tz = transformCur[5];

    float a1 = crz * sry * crx;
    float a2 = srz * srx;
    float a3 = -crz * sry * srx;
    float a4 = srz * crx;
    float a5 = srz * sry * crx;
    float a6 = -crz * srx;
    float a7 = -srz * sry * srx;
    float a8 = -crz * crx;
    float a9 = crx * cry;
    float a10 = -cry * srx;

    float b1 = -crz * sry;
    float b2 = crz * cry * srx;
    float b3 = crz * cry * crx;
    float b4 = -srz * sry;
    float b5 = srz * cry * srx;
    float b6 = srz * cry * crx;
    float b7 = -cry;
    float b8 = -sry * srx;
    float b9 = -sry * crx;

    for (int i = 0; i < pointSelNum; i++) {

        pointOri = laserCloudOri->points[i];
        coeff = coeffSel->points[i];

	float arx = ((crx*crz*sry-sry*srz)*pointOri.x + (-crz*srx-crx*sry*srz)*pointOri.y - crx*cry*pointOri.z)*coeff.y
	+ ((crx*srz+crz*srx*sry)*pointOri.x + (crx*crz-srx*sry*srz)*pointOri.y + (-cry*srx)*pointOri.z)*coeff.z;
	
	float ary = ((-crz*sry)*pointOri.x + sry*srz*pointOri.y + cry*pointOri.z)*coeff.x +
	(cry*crz*srx*pointOri.x - cry*srx*srz*pointOri.y + srx*sry*pointOri.z)*coeff.y +
	(-crx*cry*crz*pointOri.x + crx*cry*srz*pointOri.y - crx*sry*pointOri.z)*coeff.z;

        float atz = coeff.z;

        float d2 = coeff.intensity;

        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = atz;
        matB.at<float>(i, 0) = -0.05 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    //cout<<"begin solve"<<endl;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
    //cout<<"finish solve"<<endl;

    if (iterCount == 0) {
        cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[3] = {10, 10, 10};
        for (int i = 2; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 3; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }

    transformCur[0] += matX.at<float>(0, 0);
    transformCur[1] += matX.at<float>(1, 0);
    transformCur[5] += matX.at<float>(2, 0);

    for (int i = 0; i < 6; i++) {
        if (isnan(transformCur[i]))
            transformCur[i] = 0;
    }

    float deltaR = sqrt(
            pow(rad2deg(matX.at<float>(0, 0)), 2) +
            pow(rad2deg(matX.at<float>(1, 0)), 2));
    float deltaT = sqrt(
            pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1) {
        return false;
    }
    return true;
}

bool FeatureAssociation::calculateTransformationCorner(int iterCount) {
    int pointSelNum = laserCloudOri->points.size();
    //cout<<"Corner pointSelNum: "<<pointSelNum<<endl;

    if(pointSelNum<10)
      return false;
    cv::Mat matA(pointSelNum, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(3, pointSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(3, 3, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(3, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(3, 1, CV_32F, cv::Scalar::all(0));

    float srx = sin(transformCur[0]);
    float crx = cos(transformCur[0]);
    float sry = sin(transformCur[1]);
    float cry = cos(transformCur[1]);
    float srz = sin(transformCur[2]);
    float crz = cos(transformCur[2]);
    float tx = transformCur[3];
    float ty = transformCur[4];
    float tz = transformCur[5];

    float b1 = -srz * cry;
    float b2 = -srz * sry * srx - crz * crx;
    float b3 = -srz * sry * crx + crz * srx;
    float b4 = crz * cry;
    float b5 = crz * sry * srx - srz * crx;
    float b6 = crz * sry * crx + srz * srx;

    float c5 = crx * srz;
    int num_threads_ = 16;
//#pragma  omp parallel for num_threads(num_threads_) schedule(guided, 8)
    for (int i = 0; i < pointSelNum; i++) {

        pointOri = laserCloudOri->points[i];
        coeff = coeffSel->points[i];

        float arz = (-cry*srz * pointOri.x - cry*crz * pointOri.y ) * coeff.x
                    + ((crx*crz-srx*sry*srz) * pointOri.x + (-crx*srz-crz*srx*sry) * pointOri.y ) * coeff.y
                    + ((crz*srx+crx*sry*srz)*pointOri.x + (crx*crz*sry-srx*srz)*pointOri.y)* coeff.z;

        float atx = coeff.x;

        float aty = coeff.y;

        float d2 = coeff.intensity;

        matA.at<float>(i, 0) = arz;
        matA.at<float>(i, 1) = atx;
        matA.at<float>(i, 2) = aty;
        matB.at<float>(i, 0) = -0.1 * d2;
    }

    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;

    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    //sleep(1000);
    if (iterCount == 0) {
        cv::Mat matE(1, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(3, 3, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[3] = {10, 10, 10};
        for (int i = 2; i >= 0; i--) {
            if (matE.at<float>(0, i) < eignThre[i]) {
                for (int j = 0; j < 3; j++) {
                    matV2.at<float>(i, j) = 0;
                }
                isDegenerate = true;
            } else {
                break;
            }
        }
        matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
        cv::Mat matX2(3, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
    }
    transformCur[2] += matX.at<float>(0, 0);
    transformCur[3] += matX.at<float>(1, 0);
    transformCur[4] += matX.at<float>(2, 0);

    for (int i = 0; i < 6; i++) {
        if (isnan(transformCur[i])) {
            transformCur[i] = 0;
        }

    }

    float deltaR = sqrt(
            pow(rad2deg(matX.at<float>(0, 0)), 2));
    float deltaT = sqrt(
            pow(matX.at<float>(1, 0) * 100, 2) +
            pow(matX.at<float>(2, 0) * 100, 2));

    if (deltaR < 0.1 && deltaT < 0.1) {
        return false;
    }
    return true;
}


void FeatureAssociation::updateTransformation() {
    iterCount1_ = 0;
    iterCount2_ = 0;


    for (int iterCount1 = 0; iterCount1 < 25; iterCount1++) {
        laserCloudOri->clear();
        coeffSel->clear();

        findCorrespondingSurfFeatures(iterCount1);
        if (laserCloudOri->points.size() < 10)
               continue;
        if (calculateTransformationSurf(iterCount1) == false)
            break;
        iterCount1_ = iterCount1 + 1;
    }

    for (int iterCount2 = 0; iterCount2 < 25; iterCount2++) {
        laserCloudOri->clear();

        coeffSel->clear();

        findCorrespondingCornerFeatures_X(iterCount2);

        if (laserCloudOri->points.size() < 10)
            continue;

        if (calculateTransformationCorner(iterCount2) == false) {
            break;
        }
    }
}

void FeatureAssociation::toEulerAngle( Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw)
{
// roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    roll = atan2(sinr_cosp, cosr_cosp);

// pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = asin(sinp);

// yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    yaw = atan2(siny_cosp, cosy_cosp);
}

void FeatureAssociation::Matrix4dToRpyxyz(const Eigen::Matrix4f &Trans,float nums[]){

    Eigen::Quaternionf quaternion(Trans.block<3,3>(0,0));
    float roll,  pitch, yaw;
    toEulerAngle(quaternion, roll,  pitch, yaw);

    Eigen::Vector3f rpy_transformSum=Trans.block<3,3>(0,0).eulerAngles(2,1,0);
    nums[0]= roll;//rpy_transformSum(0);
    nums[1]= pitch;//rpy_transformSum(1);;
    nums[2]= yaw;//rpy_transformSum(2);//;
    nums[3]=Trans(0,3);
    nums[4]=Trans(1,3);
    nums[5]=Trans(2,3);
}

void FeatureAssociation::runFeatureAssociation(Eigen::Matrix4f Init_state_) {
    Matrix4dToRpyxyz(Init_state_,transformCur);
    updateTransformation();
}

void FeatureAssociation::RpyxyzToMatrix4f(Eigen::Matrix4f &Trans,float nums[]){
    Eigen::Vector3f eulerAngle(nums[0],nums[1],nums[2]);
    Eigen::AngleAxisf rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisf pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisf yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));

    Eigen::Quaternionf quaternion;
    quaternion=rollAngle*pitchAngle*yawAngle;
    Eigen::Matrix3f rotate(quaternion);
    Eigen::Vector3f tran(nums[3],nums[4],nums[5]);
    Trans=Eigen::Matrix4f::Identity();
    Trans.block<3,3>(0,0)=rotate;
    Trans.block<3,1>(0,3)=tran;
}

Eigen::Matrix4f FeatureAssociation::getFinalTransformation()
{
    Eigen::Matrix4f Tran;
    RpyxyzToMatrix4f(Tran,transformCur);
    return Tran;
}

