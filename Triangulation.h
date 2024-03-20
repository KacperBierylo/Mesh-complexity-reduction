#ifndef Triangulation_H
#define Triangulation_H

#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

const int K = 20; // number of nearest neighbors to use for the estimation.

pcl::PointCloud<pcl::Normal>::Ptr estimateNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

pcl::PointCloud<pcl::PointNormal>::Ptr createCloudWithNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                              const pcl::PointCloud<pcl::Normal>::Ptr& normals);

pcl::GreedyProjectionTriangulation<pcl::PointNormal> createGptObject(double radius, double mu, int maxNearestNeighbors,
                                                                     double epsAngle, double minAngle, double maxAngle);

pcl::PolygonMesh performTriangulation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

#endif //Triangulation_H
