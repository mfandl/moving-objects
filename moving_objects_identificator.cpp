#include "moving_objects_identificator.hpp"

void MovingObjectsIdentificator::setInputCloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    inputCloud1 = cloud;
}

void MovingObjectsIdentificator::setInputCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    inputCloud2 = cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MovingObjectsIdentificator::findDifference() {

    if(workingCloud->points.size() > 0) {
        workingCloud->erase(workingCloud->begin(), workingCloud->end());
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cleanedCloud1(removeLargePlanes(inputCloud1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cleanedCloud2(removeLargePlanes(inputCloud2));


    for(pcl::PointCloud<pcl::PointXYZ>::iterator it = cleanedCloud1->begin(), it2 = cleanedCloud2->begin(); it != cleanedCloud1->end(); it++, it2++) {
        if((!pcl::isFinite(*it) && pcl::isFinite(*it2)) || pcl::geometry::distance(it->getVector3fMap(), it2->getVector3fMap()) > distanceThreshold) {
            workingCloud->push_back(*it2);
        }
    }

    return workingCloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr MovingObjectsIdentificator::removeLargePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr resultCloud (cloud->makeShared());

    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(100);
    segmentation.setDistanceThreshold(0.02);

    int pointsCount = resultCloud->points.size();

    while(resultCloud->points.size() > 0.3 * pointsCount) {
        segmentation.setInputCloud(resultCloud);
        segmentation.segment(*inliers, *coefficients);
        if(inliers->indices.size() <= 50000) {
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

pcl::PointCloud<pcl::PointXYZ>::Ptr MovingObjectsIdentificator::removeOutliers() {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(workingCloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*workingCloud);

    return workingCloud;

}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > MovingObjectsIdentificator::extractClusters() {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdTree->setInputCloud(workingCloud);

    std::vector<pcl::PointIndices> indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ece;
    ece.setClusterTolerance(0.02);
    ece.setMinClusterSize(1000);
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
