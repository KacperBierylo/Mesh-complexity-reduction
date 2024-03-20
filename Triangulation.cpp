#include "Triangulation.h"

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(K);
    n.compute(*normals);

    return normals;
}

pcl::PointCloud<pcl::PointNormal>::Ptr createCloudWithNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                              const pcl::PointCloud<pcl::Normal>::Ptr& normals) {
    pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloudWithNormals);
    return cloudWithNormals;
}

pcl::GreedyProjectionTriangulation<pcl::PointNormal> createGptObject(double radius, double mu, int maxNearestNeighbors,
                                                                     double epsAngle, double minAngle, double maxAngle) {
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt;

    gpt.setSearchRadius(radius);
    gpt.setMu(mu);
    gpt.setMaximumNearestNeighbors(maxNearestNeighbors);
    gpt.setMaximumSurfaceAngle(epsAngle);
    gpt.setMinimumAngle(minAngle);
    gpt.setMaximumAngle(maxAngle);
    gpt.setNormalConsistency(false);

    return gpt;
}

pcl::PolygonMesh performTriangulation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    auto normals = estimateNormals(cloud);

    auto cloudWithNormals = createCloudWithNormals(cloud, normals);

    // create search tree
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloudWithNormals);

    // initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gpt = createGptObject(0.50, 4.5, 50, M_PI/4, M_PI/18, 2*M_PI/3);
    pcl::PolygonMesh triangles;

    // get result of triangulation
    gpt.setInputCloud(cloudWithNormals);
    gpt.setSearchMethod(tree2);
    gpt.reconstruct(triangles);

    return triangles;
}
