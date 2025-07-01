#include "scan_matching.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>  // TicToc
#include <iostream>

ScanMatchResult scanMatchingByNDT(
    const PointCloudT::Ptr& target,
    const PointCloudT::Ptr& source,
    const Pose& initialPose,
    double epsilon,
    int maxIter,
    double stepSize,
    double resolution,
    double fitnessScoreThreshold
) {
    pcl::console::TicToc timer;
    timer.tic();

    ScanMatchResult result;

    // Prepare initial guess from Pose
    Eigen::Matrix4f initialGuess = transform3D(
        initialPose.rotation.yaw,
        initialPose.rotation.pitch,
        initialPose.rotation.roll,
        initialPose.position.x,
        initialPose.position.y,
        initialPose.position.z
    ).cast<float>();

    // Configure NDT
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setTransformationEpsilon(epsilon);
    ndt.setStepSize(stepSize);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(maxIter);
    ndt.setInputSource(source);
    ndt.setInputTarget(target);

    // Perform alignment
    PointCloudT::Ptr outputCloud(new PointCloudT);
    ndt.align(*outputCloud, initialGuess);

    // Process result
    result.computationTimeMs = timer.toc();
    result.converged = ndt.hasConverged();
    result.fitnessScore = ndt.getFitnessScore();

    if (result.converged && result.fitnessScore < fitnessScoreThreshold) {
        result.transformation = ndt.getFinalTransformation().cast<double>();
        std::cout << "[NDT] Converged in " << result.computationTimeMs << " ms, "
                  << "Fitness score: " << result.fitnessScore << std::endl;
    } else {
        std::cerr << "[NDT] WARNING: Alignment failed or poor quality!" << std::endl;
        std::cerr << "[NDT] Fitness Score: " << result.fitnessScore << std::endl;
    }

    return result;
}

ScanMatchResult scanMatchingByICP(
    const PointCloudT::Ptr& target,
    const PointCloudT::Ptr& source,
    const Pose& initialPose,
    int maxIter,
    double maxCorrespondenceDistance,
    double fitnessScoreThreshold
) {
    pcl::console::TicToc timer;
    timer.tic();

    ScanMatchResult result;

    // Create the ICP object
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(maxIter);
    icp.setMaxCorrespondenceDistance(maxCorrespondenceDistance);
    icp.setInputSource(source);
    icp.setInputTarget(target);

    // Prepare initial guess from Pose.
    Eigen::Matrix4f initialGuess = transform3D(
        initialPose.rotation.yaw,
        initialPose.rotation.pitch,
        initialPose.rotation.roll,
        initialPose.position.x,
        initialPose.position.y,
        initialPose.position.z
    ).cast<float>();

    // Perform alignment
    PointCloudT::Ptr outputCloud(new PointCloudT);
    icp.align(*outputCloud, initialGuess);

    // Process result
    result.computationTimeMs = timer.toc();
    result.converged = icp.hasConverged();
    result.fitnessScore = icp.getFitnessScore();

    if (result.converged && result.fitnessScore < fitnessScoreThreshold) {
        result.transformation = icp.getFinalTransformation().cast<double>();
        std::cout << "[ICP] Converged in " << result.computationTimeMs << " ms, "
                  << "Fitness score: " << result.fitnessScore << std::endl;
    } else {
        std::cerr << "[ICP] WARNING: Alignment failed or poor quality!" << std::endl;
        std::cerr << "[ICP] Fitness Score: " << result.fitnessScore << std::endl;
        // If ICP fails, we can either return the identity or the initial guess.
    }

    return result;
}
