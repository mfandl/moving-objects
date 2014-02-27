#include <iostream>
#include <sstream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "moving_objects_identificator.hpp"
#include "classificator.hpp"

#include <pcl/pcl_macros.h>
#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
#include <pcl/console/parse.h>

using namespace std;

pcl::visualization::PCLVisualizer viewer ("Result");

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

MovingObjectsIdentificator moi;
Classificator classificator;
void findObjectsAndClassify();

void keyboard_cb(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

bool keyboardCbLock = true;

int main(int argc, char* argv[]) {

    if(argc < 3) {
        PCL_ERROR("run as ./project /path/to/cloud1/ /path/to/cloud2/ [pcd format]");
        return -1;
    }

    string fCloud1 = "";
    string fCloud2 = "";
    string models = "";
    string training = "";
    int NN;

    pcl::console::parse_argument(argc, argv, "-cloud1", fCloud1);
    pcl::console::parse_argument(argc, argv, "-cloud2", fCloud2);
    pcl::console::parse_argument(argc, argv, "-models", models);
    pcl::console::parse_argument(argc, argv, "-training", training);
    pcl::console::parse_argument(argc, argv, "-nn", NN);




    classificator.setModelsDir(models);
    classificator.setTrainingDir(training);
    classificator.setNN(NN);
    classificator.setup();

    viewer.registerKeyboardCallback(&keyboard_cb, NULL);
    viewer.setBackgroundColor(0, 0, 0);

    viewer.initCameraParameters();

    if(fCloud1 != "" && fCloud2 != "") {
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(fCloud1, *cloud1) == -1) {
            PCL_ERROR("Cloud1 reading failed\n");
            return(-1);
        }

        if(pcl::io::loadPCDFile<pcl::PointXYZ>(fCloud2, *cloud2) == -1) {
            PCL_ERROR("Cloud2 reading failed\n");
            return(-1);
        }

        findObjectsAndClassify();
    } else {
        keyboardCbLock = false;
    }

    while(!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::milliseconds(100));
    }

    return 0;
}

void keyboard_cb(const pcl::visualization::KeyboardEvent &event, void* viewer_void) {
    if(!keyboardCbLock && event.keyDown()) {
        if(event.getKeySym() == "space") {
            string fCloud1, fCloud2;
            cout << "Path to cloud1: " << endl;
            cin >> fCloud1;
            cout << "Path to cloud2: " << endl;
            cin >> fCloud2;

            if(pcl::io::loadPCDFile<pcl::PointXYZ>(fCloud1, *cloud1) == -1) {
                PCL_ERROR("Cloud1 reading failed\n");
                exit(-1);
            }

            if(pcl::io::loadPCDFile<pcl::PointXYZ>(fCloud2, *cloud2) == -1) {
                PCL_ERROR("Cloud2 reading failed\n");
                exit(-1);
            }

            findObjectsAndClassify();

        }

    }
}

void findObjectsAndClassify() {

    keyboardCbLock = true;

    viewer.removeAllPointClouds();
    viewer.removeAllShapes();

    moi.setInputClouds(cloud1, cloud2);
    moi.setEnableSceneAlignment(false);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = moi.identify();
    cout << "clusters: " << clusters.size() << endl;

    classificator.setInputClouds(clusters);
    std::vector<Classificator::ClusterClasses> classes = classificator.classify();

    viewer.addPointCloud<pcl::PointXYZ> (cloud2, "current frame");
    int clusterId = 0;
    for(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = clusters.begin(); it != clusters.end(); it++) {



        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> randomColor (*it);
        viewer.addPointCloud<pcl::PointXYZ> (*it, randomColor, "cluster" + clusterId);


        Eigen::Vector4f clusterCentroid;
        pcl::compute3DCentroid (**it, clusterCentroid);

        pcl::PointXYZ textPosition;
        textPosition.x = clusterCentroid[0];
        textPosition.y = clusterCentroid[1];
        textPosition.z = clusterCentroid[2];

        stringstream textId;
        textId << "cluster-id: " << clusterId;

        viewer.addText3D(textId.str(), textPosition, 0.015f, 1, 0, 1, textId.str(), 0);
        cout << "#" << textId.str() << endl;

        vector<string>::iterator namesIt;
        vector<float>::iterator confIt;
        for(namesIt = classes[clusterId].names.begin(), confIt = classes[clusterId].confidence.begin(); namesIt != classes[clusterId].names.end(); namesIt++, confIt++) {
            cout << "##" << *namesIt << "[" << *confIt << "]" << endl;
        }

        clusterId++;
    }

    keyboardCbLock = false;
}
