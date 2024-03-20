#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <cstdlib>
#include "DisparityMap.h"

DisparityMap::DisparityMap() {
    size = 0;
    width = 0;
    height = 0;
    scaleFactor = 0.f;
    disparityMap = nullptr;
}

void DisparityMap::readMapFromFile(const std::string& filename) {
    FILE * file = fopen(filename.c_str(), "rb");
    char auxiliaryArr[128];

    if (file != nullptr) {
        fscanf(file, "%s", auxiliaryArr);
        if (!strcmp(auxiliaryArr, "Pf")) {
            fscanf(file, "%s", auxiliaryArr);
            //width = atoi(auxiliaryArr);
            width = static_cast<int>(strtol(auxiliaryArr, nullptr, 10));

            fscanf(file, "%s", auxiliaryArr);
            //height = atoi(auxiliaryArr);
            height = static_cast<int>(strtol(auxiliaryArr, nullptr, 10));

            size = width * height;

            fscanf(file, "%s", auxiliaryArr);
            scaleFactor = static_cast<float>(atof(auxiliaryArr));

            fseek(file, 0, SEEK_END);
            long tempSize = ftell(file);
            long pos = tempSize - width * height * sizeof(float);
            fseek(file, pos, SEEK_SET);

            auto* auxiliaryMap = new float[size];
            fread(auxiliaryMap, sizeof(float), size, file);
            fclose(file);

            disparityMap = new float[size];
            for (int i = 0; i < height; ++i) {
                memcpy(&disparityMap[(height - i - 1) * width], &auxiliaryMap[i * width], width * sizeof(float));
            }

            delete[] auxiliaryMap;
        }
    } else {
        throw std::runtime_error("File doesn't exists!");
    }
}

int DisparityMap::getSize() const {
    return size;
}

int DisparityMap::getWidth() const {
    return width;
}

int DisparityMap::getHeight() const {
    return height;
}

float *DisparityMap::getDisparityMap() {
    return disparityMap;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr DisparityMap::convertToPointCloud() {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    float* map = this->getDisparityMap();

    float x = -DEFAULT_DIFFERENCE, y = (static_cast<float>(this->getHeight()));
    for (int i = 0; i < this->getSize(); ++i) {
        if (i % this->getWidth() == 0) {
            x = 0.0f;
            y -= DEFAULT_DIFFERENCE;
        }
        x += DEFAULT_DIFFERENCE;

        float z = E * (1 / map[i]);
        cloud->push_back(pcl::PointXYZ(static_cast<float>(x), static_cast<float>(y), z));
    }

    return cloud;
}

