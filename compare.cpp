#include <iostream>
#include <sstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "comparator.hpp"

using namespace std;

pcl::visualization::CloudViewer viewer ("FRAME 1");
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZRGBA>);

Comparator comparator(0.1f);

int main(int argc, char* argv[]) {

    if(argc < 3) {
        return -1;
    }

    string fCloud1 = string(argv[1]);
    string fCloud2 = string(argv[2]);

    cout << "arg1 - " << fCloud1 << endl;
    cout << "arg2 - " << fCloud2 << endl;

    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fCloud1, *cloud1) == -1) {
        PCL_ERROR("File reading failed\n");
        return(-1);
    }

    if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fCloud2, *cloud2) == -1) {
        PCL_ERROR("File reading failed\n");
        return(-1);
    }

    comparator.setCloud1(cloud1);
    comparator.setCloud2(cloud2);

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr moved = comparator.findMovedPoints();
    moved = comparator.removeOutliers(moved);
    std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> clusters = comparator.extractClusters(moved);
    cout << "clusters: " << clusters.size() << endl;
    viewer.showCloud(moved);
    while(!viewer.wasStopped());

    return 0;
}
