#ifndef DESERIALIZER_H
#define DESERIALIZER_H

#include <fstream>
#include <memory>

#include "json.hpp"

struct DistanceSensorData
{
    double angle;  // in degrees.
    double distance;
};
struct Forces
{
    std::vector<DistanceSensorData> repulsiveFieldData;
    std::vector<DistanceSensorData> attrFieldData;
    std::vector<DistanceSensorData> totalFieldData;
};

using LidarSample = std::vector<DistanceSensorData>;

struct Data
{
    std::vector<Forces> forces;
    std::vector<LidarSample> lidar_samples;
};

using json = nlohmann::json;

class Deserializer
{
public:
    Deserializer();
    Data deserializeData(const std::string& filename);
};

#endif // DESERIALIZER_H
