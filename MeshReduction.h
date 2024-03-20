#ifndef MeshReduction_H
#define MeshReduction_H

#include <pcl/PolygonMesh.h>

pcl::PolygonMesh::Ptr reduceMesh(pcl::PolygonMesh mesh, float reductionFactor);

pcl::PolygonMesh smoothMesh(const pcl::PolygonMesh::Ptr& mesh, int iterationsNumber);

#endif //MeshReduction_H