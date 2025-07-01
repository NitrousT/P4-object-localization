#ifndef SCAN_MATCHING_H
#define SCAN_MATCHING_H

#include "helper.h"

struct ScanMatchResult {
    Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
    double fitnessScore = -1.0;
    bool converged = false;
    double computationTimeMs = 0.0;
};
// NDT and ICP declaration
ScanMatchResult scanMatchingByNDT(
    const PointCloudT::Ptr& target,
    const PointCloudT::Ptr& source,
    const Pose& initialPose,
    double epsilon = 1e-8,
    int maxIter = 60,
    double stepSize = 0.1,
    double resolution = 2.0,
    double fitnessScoreThreshold = 1.5
);

ScanMatchResult scanMatchingByICP(
    const PointCloudT::Ptr& target,
    const PointCloudT::Ptr& source,
    const Pose& initialPose,
    int maxIter = 10,
    double maxCorrespondenceDistance = 1.0, 
    double fitnessScoreThreshold = 0.5
);

#endif
