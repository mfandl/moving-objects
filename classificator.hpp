#ifndef CLASSIFICATOR_HPP
#define CLASSIFICATOR_HPP

#include <pcl/pcl_macros.h>
#include <pcl/apps/3d_rec_framework/pipeline/global_nn_classifier.h>
#include <pcl/apps/3d_rec_framework/pc_source/mesh_source.h>
#include <pcl/apps/3d_rec_framework/feature_wrapper/global/esf_estimator.h>
#include <pcl/apps/3d_rec_framework/utils/metrics.h>
#include <pcl/console/parse.h>

class Classificator {
private:
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    std::string modelsDir;
    std::string trainingDir;
    std::string descriptorName;
    int nn;
    pcl::rec_3d_framework::GlobalNNPipeline<flann::L1, pcl::PointXYZ, pcl::ESFSignature640> global;
public:

    class ClusterClasses {
        public:
            std::vector<std::string> names;
            std::vector<float> confidence;
    };

    Classificator();

    void setInputClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters) {this->clusters = clusters;};
    void setup();

    std::vector<ClusterClasses> classify();
    void setModelsDir(std::string dir) {
        modelsDir = dir;
    }

    void setTrainingDir(std::string dir) {
        trainingDir = dir;
    }

    void setNN (int nn) {
        this->nn = nn;
    }

};

#endif
