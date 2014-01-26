#ifndef COMPARATOR_H
#define COMPARATOR_H
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

class Comparator {
private:
    float distanceThreshold;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2;
public:
    Comparator() : distanceThreshold(0.01) {}
    Comparator(float distanceThreshold) : distanceThreshold(distanceThreshold) {}

    void setCloud1(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    void setCloud2(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr findMovedPoints();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr removeLargePlanes(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr removeOutliers(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr > extractClusters(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);

};

#endif
