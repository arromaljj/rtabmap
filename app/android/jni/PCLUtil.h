#ifndef PCLUTIL_H
#define PCLUTIL_H

#include <rtabmap/core/LaserScan.h>
#include <vector>
#include <string>
#include <cstdint>

// Structure to hold the serialized PointCloud2 data
struct SerializedPointCloud2 {
    std::vector<uint8_t> data;  // The serialized point cloud data
    std::string frame_id;       // Frame ID
    uint64_t timestamp;         // Timestamp in nanoseconds
};

/**
 * @brief Serializes a LaserScan into a binary format
 * @param scan Input point cloud as LaserScan
 * @return vector of bytes containing the serialized data
 * @throw std::runtime_error if input scan is empty
 */
std::vector<uint8_t> serializeLaserScan(const rtabmap::LaserScan& scan);

/**
 * @brief Deserializes a binary format back into a LaserScan
 * @param data Input binary data
 * @return LaserScan object
 * @throw std::runtime_error if data is corrupted or invalid
 */
rtabmap::LaserScan deserializeLaserScan(const std::vector<uint8_t>& data);

/**
 * @brief Converts a LaserScan point cloud to a ROS 2 PointCloud2-compatible binary message
 * @param scan Input point cloud as LaserScan
 * @param frame_id The frame ID for the point cloud
 * @param timestamp_ns Timestamp in nanoseconds
 * @return SerializedPointCloud2 containing the binary message
 * @throw std::runtime_error if input scan is empty
 */
SerializedPointCloud2 convertLaserScanToSerializedPointCloud2(
    const rtabmap::LaserScan& scan,
    const std::string& frame_id,
    uint64_t timestamp_ns);

#endif // PCLUTIL_H 