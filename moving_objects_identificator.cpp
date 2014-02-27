#include "moving_objects_identificator.hpp"

MovingObjectsIdentificator::MovingObjectsIdentificator() :
    workingCloud (new pcl::PointCloud<pcl::PointXYZ>),
    differenceDistanceThreshold(0.01),
    ransacDistanceThreshold(0.02),
    ransacMaxIterations(100),
    largePlaneMinimumSize(50000),
    meanK(50),
    stddevMulThresh(1.0),
    clusterTolerance(0.02),
    minClusterSize(1000),
    enableSceneAlignment(true),
    ICPMaxIterations(10),
    verbose(false) {}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> MovingObjectsIdentificator::identify() {
    if(verbose) std::cout << "\nMoving Object Identification start" << std::endl;
    findDifference();
    removeOutliers();
    if(verbose) std::cout << "Moving Object Identification end" << std::endl;
    return extractClusters();

}

void MovingObjectsIdentificator::findDifference() {
    if(verbose) std::cout << "Finding difference in clouds ... " << std::endl;
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
    if(verbose) std::cout << "Finding difference in clouds done!" << std::endl;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr MovingObjectsIdentificator::removeLargePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    if(verbose) std::cout << "Removing large planes ... ";
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

    if(verbose) std::cout << "done!" << std::endl;

    return resultCloud;
}

void MovingObjectsIdentificator::removeOutliers() {
    if(verbose) std::cout << "Removing outliers ... ";
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(workingCloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddevMulThresh);
    sor.filter(*workingCloud);
    if(verbose) std::cout << "done!" << std::endl;

}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> MovingObjectsIdentificator::extractClusters() {
    if(verbose) std::cout << "Extracting clusters ... ";
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

    if(verbose) std::cout << "done!" << std::endl;

    return clusters;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr MovingObjectsIdentificator::align() {
    if(verbose) std::cout << "Aligning scenes ... ";
    //removing NaN's because it causes align boudary error (pcl git commit hash: cfd04b3)
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedCloud1 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr fixedCloud2 (new pcl::PointCloud<pcl::PointXYZ>);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*inputCloud1, *fixedCloud1, indices);
    pcl::removeNaNFromPointCloud(*inputCloud2, *fixedCloud2, indices);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(fixedCloud1);
    icp.setInputTarget(fixedCloud2);
    icp.setMaximumIterations(ICPMaxIterations);

    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned (new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*aligned);

    if(verbose) std::cout << (icp.hasConverged() ? "done! " : "dailed! ") << " score: " << icp.getFitnessScore() << std::endl;

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

float MovingObjectsIdentificator::getDifferenceDistanceThreshold() {
    return differenceDistanceThreshold;
}

void MovingObjectsIdentificator::setDifferenceDistanceTreshold(float thr){
    differenceDistanceThreshold = thr;
}

float MovingObjectsIdentificator::getRansacDistanceThreshold() {
    return ransacDistanceThreshold;
}

void MovingObjectsIdentificator::setRansacDistanceThreshold(float thr) {
    ransacDistanceThreshold = thr;
}

int MovingObjectsIdentificator::getRansacMaxIterations() {
    return ransacMaxIterations;
}

void MovingObjectsIdentificator::setRansacMaxIterations(int max) {
    ransacMaxIterations = max;
}

int MovingObjectsIdentificator::getLargePlaneMinimumSize() {
    return largePlaneMinimumSize;
}

void MovingObjectsIdentificator::setLargePlaneMinimumSize(int size) {
    largePlaneMinimumSize = size;
}

int MovingObjectsIdentificator::getMeanK() {
    return meanK;
}

void MovingObjectsIdentificator::setMeanK(int meanK) {
    this->meanK = meanK;
}

float MovingObjectsIdentificator::getStddevMulThresh() {
    return stddevMulThresh;
}
void MovingObjectsIdentificator::setStddevMulThresh(float thr) {
    stddevMulThresh = thr;
}

float MovingObjectsIdentificator::getClusterTolerance() {
    return clusterTolerance;
}

void MovingObjectsIdentificator::setClusterTolerance(int tolerance) {
    clusterTolerance = tolerance;
}

int MovingObjectsIdentificator::getMinClusterSize() {
    return minClusterSize;
}

void MovingObjectsIdentificator::setMinClusterSize(int size) {
    minClusterSize = size;
}

bool MovingObjectsIdentificator::getEnableSceneAlignment() {
    return enableSceneAlignment;
}

void MovingObjectsIdentificator::setEnableSceneAlignment(bool enable) {
    enableSceneAlignment = enable;
}

int MovingObjectsIdentificator::getICPMaxIterations() {
    return ICPMaxIterations;
}

void MovingObjectsIdentificator::setICPMaxIterations(int iterations) {
    ICPMaxIterations = iterations;
}

void MovingObjectsIdentificator::setVerbose(bool verbose) {
    this->verbose = verbose;
}
