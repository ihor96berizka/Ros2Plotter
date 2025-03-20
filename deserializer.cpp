#include "deserializer.h"
#include <QDebug>
namespace
{
inline std::string kAngleKey{"angle"};
inline std::string kMagnitudeKey{"magnitude"};
inline std::string kRepulsiveKey{"repulsive"};
inline std::string kAttractiveKey{"attractive"};
inline std::string kTotalKey{"total"};
inline std::string kForcesKey{"forces"};
inline std::string kLidarKey{"lidar"};
inline std::string kLidarSamplesKey{"samples"};
inline std::string kDistanceKey{"distance"};
}

/**
 *
 * filename.json example
 *
 * forces object
 * {
 *  "repulsive": [
 *          {
 *              "angle": 0,
 *              "magnitude" : 1
 *          },
 *          {
 *              "angle": 0,
 *              "magnitude" : 1
 *          },
 *          ...
 *      ],
 *   "attractive": [],
 *   "total" : []
 * }
 *
 * main json object:
 * {
 *  "forces" = [
 *          forces_object1,
 *          ...
 *      ]
 * }
 */


Deserializer::Deserializer() {}

Data Deserializer::deserializeData(const std::string &filename)
{
    Data data;
    std::ifstream f(filename);
    json raw_data = json::parse(f);

    auto forces_array = raw_data[kForcesKey];

    for (const auto& force: forces_array)
    {
        // parsing force obj
        qDebug() << "parsing Force obj....";
        //qDebug() << "repulsive:";
        Forces current_force;
        auto repulsive_array = force[kRepulsiveKey];
        for (const auto& item: repulsive_array)
        {
            current_force.repulsiveFieldData.emplace_back(DistanceSensorData{item[kAngleKey], item[kMagnitudeKey]});
            //qDebug() << "pushed angle:" << static_cast<double>(item[kAngleKey]) << " | magnitude: " << static_cast<double>(item[kMagnitudeKey]);
        }

        //qDebug() << "attr:";
        auto attractive_array = force[kAttractiveKey];
        for (const auto& item: attractive_array)
        {
            current_force.attrFieldData.emplace_back(DistanceSensorData{item[kAngleKey], item[kMagnitudeKey]});
            //qDebug() << "pushed angle:" << static_cast<double>(item[kAngleKey]) << " | magnitude: " << static_cast<double>(item[kMagnitudeKey]);
        }

        //qDebug() << "total:";
        auto total_array = force[kTotalKey];
        for (const auto& item: total_array)
        {
            current_force.totalFieldData.emplace_back(DistanceSensorData{item[kAngleKey], item[kMagnitudeKey]});
            //qDebug() << "pushed angle:" << static_cast<double>(item[kAngleKey]) << " | magnitude: " << static_cast<double>(item[kMagnitudeKey]);
        }

        data.forces.push_back(std::move(current_force));
    }

    auto lidar_array = raw_data[kLidarKey];

    for (const auto& sample : lidar_array)
    {
        qDebug() << "parsing lidar sample obj....";
        LidarSample current_sample;
        auto sample_array = sample[kLidarSamplesKey];
        for (const auto& item: sample_array)
        {
            current_sample.emplace_back(DistanceSensorData{item[kAngleKey], item[kDistanceKey]});
        }

        data.lidar_samples.push_back(std::move(current_sample));
    }

    return data;
}
