#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "comparator.hpp"

using namespace std;

pcl::visualization::CloudViewer viewer1 ("FRAME 1");
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

Comparator comparator(0.1f);

int main() {

    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>("frame1.pcd", *cloud1) == -1) {
        PCL_ERROR("File reading failed\n");
        return(-1);
    }

    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>("frame2.pcd", *cloud2) == -1) {
        PCL_ERROR("File reading failed\n");
        return(-1);
    }

    comparator.setCloud1(cloud1);
    comparator.setCloud2(cloud2);

    //viewer1.showCloud(comparator.removeLargePlanes(cloud2));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr moved = comparator.findMovedPoints();
    moved = comparator.removeOutliers(moved);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters = comparator.extractClusters(moved);
    //int clustersCount = clusters.size();
    cout << "clusters: " << clusters.size() << endl;
    viewer1.showCloud(clusters[0]);
    while(!viewer1.wasStopped());

    return 0;
}
