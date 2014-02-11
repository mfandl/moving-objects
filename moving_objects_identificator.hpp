#ifndef MOI_HPP
#define MOI_HPP
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/geometry.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

class MovingObjectsIdentificator {
private:
    float distanceThreshold;

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr workingCloud;
public:
    MovingObjectsIdentificator() : distanceThreshold(0.01), workingCloud (new pcl::PointCloud<pcl::PointXYZ>) {}
    MovingObjectsIdentificator(float distanceThreshold) : distanceThreshold(distanceThreshold), workingCloud (new pcl::PointCloud<pcl::PointXYZ>) {}

    void setInputCloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setInputCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr findDifference();
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeLargePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > extractClusters();

};

#endif
