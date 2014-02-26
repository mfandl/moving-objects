#include "classificator.hpp"

Classificator::Classificator() : descriptorName ("esf") {};

void Classificator::setup() {
    boost::shared_ptr<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > mesh_source (new pcl::rec_3d_framework::MeshSource<pcl::PointXYZ>);
    mesh_source->setPath(modelsDir);
    mesh_source->setResolution(150);
    mesh_source->setTesselationLevel(1);
    mesh_source->setViewAngle(57.f);
    mesh_source->setRadiusSphere(1.5f);
    mesh_source->setModelScale(1.f);
    mesh_source->generate(trainingDir);

    boost::shared_ptr<pcl::rec_3d_framework::Source<pcl::PointXYZ> > cast_source;
    cast_source = boost::static_pointer_cast<pcl::rec_3d_framework::MeshSource<pcl::PointXYZ> > (mesh_source);

    boost::shared_ptr<pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal> > normal_estimator;
    normal_estimator.reset (new pcl::rec_3d_framework::PreProcessorAndNormalEstimator<pcl::PointXYZ, pcl::Normal>);
    normal_estimator->setCMR (true);
    normal_estimator->setDoVoxelGrid(true);
    normal_estimator->setRemoveOutliers(true);
    normal_estimator->setFactorsForCMR(3, 7);

    boost::shared_ptr<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> > estimator;
    estimator.reset(new pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640>);

    boost::shared_ptr<pcl::rec_3d_framework::GlobalEstimator<pcl::PointXYZ, pcl::ESFSignature640> > cast_estimator;
    cast_estimator = boost::dynamic_pointer_cast<pcl::rec_3d_framework::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> > (estimator);

    global.setDataSource(cast_source);
    global.setTrainingDir(trainingDir);
    global.setDescriptorName(descriptorName);
    global.setFeatureEstimator(cast_estimator);
    global.setNN(nn);
    global.initialize(false);

}

std::vector<Classificator::ClusterClasses> Classificator::classify() {
    std::vector<Classificator::ClusterClasses> clusterClassesVector;
    for(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>::iterator it = clusters.begin(); it != clusters.end(); it++) {
        global.setInputCloud(*it);
        global.classify();

        Classificator::ClusterClasses clusterClasses;
        global.getCategory(clusterClasses.names);
        global.getConfidence(clusterClasses.confidence);

        clusterClassesVector.push_back(clusterClasses);
    }

    return clusterClassesVector;
}
