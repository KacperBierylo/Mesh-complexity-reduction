#ifndef Visualisation_H
#define Visualisation_H

#include <pcl/PolygonMesh.h>
#include "DisparityMap.h"

void drawPolygonsInVisualizer(const pcl::PolygonMesh& mesh, const pcl::PolygonMesh& reducedMesh);

#endif //Visualisation_H