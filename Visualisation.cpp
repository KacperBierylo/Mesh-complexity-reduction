#include "Visualisation.h"
#include <pcl/visualization/pcl_visualizer.h>

void drawPolygonsInVisualizer(const pcl::PolygonMesh &mesh, const pcl::PolygonMesh &reducedMesh) {
    pcl::visualization::PCLVisualizer::Ptr meshViewer;
    meshViewer.reset(new pcl::visualization::PCLVisualizer);
    meshViewer->addPolygonMesh(mesh, "mesh");
    meshViewer->setWindowName("Polygon mesh - visualisation");
    meshViewer->setShowFPS(false);

    std::string text1 = "Number of triangles: ";
    text1 += std::to_string(mesh.polygons.size());
    meshViewer->addText(text1, 10, 10);

    pcl::visualization::PCLVisualizer::Ptr reducedMeshViewer;
    reducedMeshViewer.reset(new pcl::visualization::PCLVisualizer);
    reducedMeshViewer->addPolygonMesh(reducedMesh, "reducedMesh");
    reducedMeshViewer->setWindowName("Reduced polygon mesh - visualisation");
    reducedMeshViewer->setShowFPS(false);

    std::string text2 = "Number of triangles: ";
    text2 += std::to_string(reducedMesh.polygons.size());
    reducedMeshViewer->addText(text2, 10, 10);

    meshViewer->spin();
    reducedMeshViewer->spin();
}