#include <iostream>
#include <sstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "comparator.hpp"

using namespace std;

pcl::visualization::PCLVisualizer viewer ("Result");
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


    viewer.setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> cloudColor(cloud2);
    viewer.addPointCloud<pcl::PointXYZRGBA> (cloud2, cloudColor, "current frame");


    int clusterNum = 0;
    for(std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>::iterator it = clusters.begin(); it != clusters.end(); it++) {
        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBA> randomColor (*it);
        viewer.addPointCloud<pcl::PointXYZRGBA> (*it, randomColor, "cluster" + clusterNum);
        clusterNum++;
    }


    viewer.initCameraParameters();

    while(!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::milliseconds(100));
    }

    return 0;
}
