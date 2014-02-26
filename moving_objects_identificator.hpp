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
#include <pcl/registration/icp.h>

class MovingObjectsIdentificator {
private:
    float differenceDistanceThreshold;
    float ransacDistanceThreshold;
    int ransacMaxIterations;
    int largePlaneMinimumSize;
    int meanK;
    float stddevMulThresh;
    float clusterTolerance;
    int minClusterSize;
    bool enableSceneAlignment;

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr workingCloud;

    void findDifference();
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeLargePlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr);
    void removeOutliers();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> extractClusters();
    pcl::PointCloud<pcl::PointXYZ>::Ptr align();
public:
    MovingObjectsIdentificator();
    void setInputCloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setInputCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setInputClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> identify();


};

#endif
