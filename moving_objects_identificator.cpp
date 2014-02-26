#include "moving_objects_identificator.hpp"

MovingObjectsIdentificator::MovingObjectsIdentificator() :
    workingCloud (new pcl::PointCloud<pcl::PointXYZ>),
    differenceDistanceThreshold(0.01),
    ransacDistanceThreshold(0.02),
    ransacMaxIterations(100),
    largePlaneMinimumSize(50000),
    meanK(50),
    stddevMulThresh(1.0),
    clusterTolerance(0.2),
    minClusterSize(1000),
    enableSceneAlignment(true) {}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> MovingObjectsIdentificator::identify() {
    findDifference();
    removeOutliers();
    return extractClusters();
}

void MovingObjectsIdentificator::findDifference() {
    if(workingCloud->points.size() > 0) {
        workingCloud->erase(workingCloud->begin(), workingCloud->end());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cleanedCloud1;
    if(enableSceneAlignment) {
        cleanedCloud1 = removeLargePlanes(align());
    } else {
        cleanedCloud1 = removeLargePlanes(inputCloud1);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cleanedCloud2(removeLargePlanes(inputCloud2));

    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cleanedCloud1->begin(), it2 = cleanedCloud2->begin(); it != cleanedCloud1->end(); it++, it2++) {
        if((!pcl::isFinite(*it) && pcl::isFinite(*it2)) || pcl::geometry::distance(it->getVector3fMap(), it2->getVector3fMap()) > differenceDistanceThreshold) {
            workingCloud->push_back(*it2);
        }
    }

}

pcl::PointCloud<pcl::PointXYZ>::Ptr MovingObjectsIdentificator::removeLargePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud (cloud->makeShared());
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(ransacMaxIterations);
    segmentation.setDistanceThreshold(ransacDistanceThreshold);

    int pointsCount = resultCloud->points.size();
    while(resultCloud->points.size() > 0.3 * pointsCount) {
        segmentation.setInputCloud(resultCloud);
        segmentation.segment(*inliers, *coefficients);

        if(inliers->indices.size() <= largePlaneMinimumSize) {
            break;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(resultCloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filterDirectly(resultCloud);
    }

    return resultCloud;
}

void MovingObjectsIdentificator::removeOutliers() {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(workingCloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*workingCloud);
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> MovingObjectsIdentificator::extractClusters() {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdTree->setInputCloud(workingCloud);

    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setClusterTolerance(clusterTolerance);
    ece.setMinClusterSize(minClusterSize);
    ece.setSearchMethod(kdTree);
    ece.setInputCloud(workingCloud);
    ece.extract(indices);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > clusters;
    for(std::vector<pcl::PointIndices>::iterator it = indices.begin(); it != indices.end(); it++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(workingCloud);

        pcl::PointIndices::Ptr pi(new pcl::PointIndices);
        pi->indices=it->indices;
        extract.setIndices(pi);
        extract.setNegative(false);
        extract.filter(*cluster);
        clusters.push_back(cluster);
    }

    return clusters;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MovingObjectsIdentificator::align() {

    //removing NaN's because it causes align boudary error (pcl git commit hash: cfd04b3)
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedCloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedCloud2 (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*inputCloud1, *fixedCloud1, indices);
    pcl::removeNaNFromPointCloud(*inputCloud2, *fixedCloud2, indices);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(fixedCloud1);
    icp.setInputTarget(fixedCloud2);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*aligned);

    return aligned;
}

void MovingObjectsIdentificator::setInputCloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    inputCloud1 = cloud;
}

void MovingObjectsIdentificator::setInputCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    inputCloud2 = cloud;
}

void MovingObjectsIdentificator::setInputClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
    inputCloud1 = cloud1;
    inputCloud2 = cloud2;
}
