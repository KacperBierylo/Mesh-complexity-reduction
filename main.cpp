#include <pcl/io/vtk_io.h>
#include "DisparityMap.h"
#include "Triangulation.h"
#include "Visualisation.h"
#include "MeshReduction.h"

std::string grabExtension(const std::string& filename) {

    int index = filename.rfind('.', filename.length());
    if (index != std::string::npos) {
        return(filename.substr(index + 1 , filename.length() - index));
    }

    return("");
}

char* openFileChooser() {
    const char zenity[] = "/usr/bin/zenity";
    static char result[256];

    sprintf(result,"%s  --file-selection --modal --title=\"%s\" ", zenity, "Select .pfm file");

    FILE *f = popen(result,"r");
    fgets(result, 256, f);

    return result;
}

std::string getFilename() {
    std::string filename;
    char* result = openFileChooser();

    for (int i = 0; i < 256; ++i) {
        if (result[i] == '\0')
            break;

        filename += result[i];
    }

    filename.pop_back();
    return filename;
}

int main() {
    std::string filename = getFilename();
    if (grabExtension(filename) != "pfm") {
        std::cout << "Invalid file selected!" << '\n';
        return 0;
    }

    DisparityMap disparityMap = DisparityMap();
    try {
        disparityMap.readMapFromFile(filename);
    } catch (const std::exception& e) {
        std::cout << e.what() << '\n';
        return 0;
    }

    auto cloud = disparityMap.convertToPointCloud();

    auto mesh = performTriangulation(cloud);

    auto reducedMesh = reduceMesh(mesh, 0.9);

    auto smoothedReducedMesh = smoothMesh(reducedMesh, 100);

    drawPolygonsInVisualizer(mesh, smoothedReducedMesh);

    return 0;
}
