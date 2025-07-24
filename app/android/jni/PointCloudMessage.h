#pragma once
#include "Message.h"
#include <opencv2/core/core.hpp>
#include <vector>

class PointCloudMessage : public Message {
public:
    PointCloudMessage(const std::string& topic = "rtabmap/cloud", const std::string& frameId = "map");
    ~PointCloudMessage();

    // Set point cloud data from OpenCV matrix (Nx3 or Nx6 with RGB)
    void setPointCloud(const cv::Mat& points);
    
    // Create JSON payload (sensor_msgs/PointCloud2 format)
    virtual std::string createJsonPayload() const;
    
    // Structure for PointCloud2 message field
    struct PointField {
        std::string name;
        uint32_t offset;
        uint8_t datatype;
        uint32_t count;
        
        // ROS point field datatypes
        static const uint8_t INT8 = 1;
        static const uint8_t UINT8 = 2;
        static const uint8_t INT16 = 3;
        static const uint8_t UINT16 = 4;
        static const uint8_t INT32 = 5;
        static const uint8_t UINT32 = 6;
        static const uint8_t FLOAT32 = 7;
        static const uint8_t FLOAT64 = 8;
    };
    
    // Clear point cloud data
    void clear();
    
    // Getters
    size_t getNumPoints() const { return points_.rows; }
    bool hasColors() const { return hasColors_; }
    
private:
    cv::Mat points_;           // Points matrix (Nx3 or Nx6 with RGB)
    bool hasColors_{false};    // Whether the points have RGB data
    int height_{1};            // Default to unorganized point cloud (height=1)
    int width_{0};             // Width calculated based on number of points
    
    // Helper methods
    std::vector<PointField> createPointFields() const;
    std::string pointCloudToBase64() const;
}; 