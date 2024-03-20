#ifndef DisparityMap_H
#define DisparityMap_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

class DisparityMap {
private:
    const float E = 100.0f; // for transformations, the value may be different
    const float DEFAULT_DIFFERENCE = 0.001f;
    int size;
    int width;
    int height;
    float scaleFactor; //also called endianness
    float* disparityMap;

public:
    DisparityMap();
    void readMapFromFile(const std::string& filename);
    int getSize() const;
    int getWidth() const;
    int getHeight() const;
    float * getDisparityMap();
    pcl::PointCloud<pcl::PointXYZ>::Ptr convertToPointCloud();
};

#endif //DisparityMap_H

