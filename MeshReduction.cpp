#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include "MeshReduction.h"

pcl::PolygonMesh::Ptr reduceMesh(pcl::PolygonMesh mesh, float reductionFactor) {
    pcl::MeshQuadricDecimationVTK meshQuadricDecimationVtk = pcl::MeshQuadricDecimationVTK();
    pcl::PolygonMesh::Ptr trianglesPtr(&mesh);
    meshQuadricDecimationVtk.setInputMesh(trianglesPtr);
    meshQuadricDecimationVtk.setTargetReductionFactor(reductionFactor);

    pcl::PolygonMesh::Ptr reducedMesh(new pcl::PolygonMesh());
    meshQuadricDecimationVtk.process(*reducedMesh);

    return reducedMesh;
}

pcl::PolygonMesh smoothMesh(const pcl::PolygonMesh::Ptr& mesh, int iterationsNumber) {
    pcl::MeshSmoothingLaplacianVTK laplacianVtk = pcl::MeshSmoothingLaplacianVTK();
    laplacianVtk.setInputMesh(mesh);
    laplacianVtk.setNumIter(iterationsNumber);
    laplacianVtk.setConvergence(0.5f);
    laplacianVtk.setRelaxationFactor(0.25f);
    laplacianVtk.setFeatureAngle(360.f);
    laplacianVtk.setEdgeAngle(180.f);

    pcl::PolygonMesh smoothedMesh;
    laplacianVtk.process(smoothedMesh);

    return smoothedMesh;
}