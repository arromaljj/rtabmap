#include "PointCloudMessage.h"
#include <iostream>
#include <sstream>

// For iOS, we'll use a simpler logging system
#define LOG_TAG "PointCloudMessage"
#define LOGI(msg) std::cout << "[INFO][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGW(msg) std::cout << "[WARN][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGE(msg) std::cout << "[ERROR][" << LOG_TAG << "] " << (msg) << std::endl

PointCloudMessage::PointCloudMessage(const std::string& topic, const std::string& frameId)
    : Message(topic, frameId)
{
}

PointCloudMessage::~PointCloudMessage()
{
}

void PointCloudMessage::setPointCloud(const cv::Mat& points)
{
    // Check that the input is valid (Nx3 or Nx6 with float or double values)
    if (points.empty() || (points.cols != 3 && points.cols != 6) || 
        (points.type() != CV_32FC1 && points.type() != CV_64FC1)) {
        LOGE("Invalid point cloud format. Expected Nx3 or Nx6 with float or double values");
        return;
    }
    
    // Store the points and update metadata
    points_ = points;
    hasColors_ = (points.cols == 6);
    width_ = points.rows;
    
    LOGI("Point cloud set with " + std::to_string(points.rows) + 
         " points, with colors = " + std::string(hasColors_ ? "true" : "false"));
}

void PointCloudMessage::clear()
{
    points_.release();
    hasColors_ = false;
    width_ = 0;
}

std::vector<PointCloudMessage::PointField> PointCloudMessage::createPointFields() const
{
    std::vector<PointField> fields;
    
    // X, Y, Z fields
    PointField x {"x", 0, PointField::FLOAT32, 1};
    PointField y {"y", 4, PointField::FLOAT32, 1};
    PointField z {"z", 8, PointField::FLOAT32, 1};
    
    fields.push_back(x);
    fields.push_back(y);
    fields.push_back(z);
    
    // Add RGB fields if the point cloud has colors
    if (hasColors_) {
        PointField r {"r", 12, PointField::UINT8, 1};
        PointField g {"g", 13, PointField::UINT8, 1};
        PointField b {"b", 14, PointField::UINT8, 1};
        
        fields.push_back(r);
        fields.push_back(g);
        fields.push_back(b);
    }
    
    return fields;
}

std::string PointCloudMessage::pointCloudToBase64() const
{
    if (points_.empty()) {
        return "";
    }
    
    // Calculate point step (bytes per point)
    int pointStep = hasColors_ ? 16 : 12;  // XYZ (12 bytes) + RGB (3 bytes) + 1 byte padding = 16 bytes
    
    // Prepare buffer for converted data
    size_t dataSize = points_.rows * pointStep;
    std::vector<uint8_t> buffer(dataSize, 0);
    
    // Convert the points to the ROS expected format
    for (int i = 0; i < points_.rows; ++i) {
        float x, y, z;
        if (points_.type() == CV_32FC1) {
            x = points_.at<float>(i, 0);
            y = points_.at<float>(i, 1);
            z = points_.at<float>(i, 2);
        } else {
            x = static_cast<float>(points_.at<double>(i, 0));
            y = static_cast<float>(points_.at<double>(i, 1));
            z = static_cast<float>(points_.at<double>(i, 2));
        }
        
        // Calculate offset for this point
        size_t offset = i * pointStep;
        
        // Copy XYZ coordinates (floats = 4 bytes each)
        memcpy(&buffer[offset], &x, sizeof(float));
        memcpy(&buffer[offset + 4], &y, sizeof(float));
        memcpy(&buffer[offset + 8], &z, sizeof(float));
        
        // Copy RGB values if available
        if (hasColors_) {
            uint8_t r, g, b;
            if (points_.type() == CV_32FC1) {
                r = static_cast<uint8_t>(points_.at<float>(i, 3) * 255.0f);
                g = static_cast<uint8_t>(points_.at<float>(i, 4) * 255.0f);
                b = static_cast<uint8_t>(points_.at<float>(i, 5) * 255.0f);
            } else {
                r = static_cast<uint8_t>(points_.at<double>(i, 3) * 255.0);
                g = static_cast<uint8_t>(points_.at<double>(i, 4) * 255.0);
                b = static_cast<uint8_t>(points_.at<double>(i, 5) * 255.0);
            }
            
            buffer[offset + 12] = r;
            buffer[offset + 13] = g;
            buffer[offset + 14] = b;
            // buffer[offset + 15] is padding (already zeroed)
        }
    }
    
    // Base64 encode the data
    return base64_encode(buffer.data(), buffer.size());
}

// UPDATED: Renamed method and returns only inner JSON payload
std::string PointCloudMessage::createJsonPayload() const
{
    std::stringstream ss;

    // Get timestamp
    uint32_t sec = std::time(nullptr);

    // Get point cloud fields
    auto fields = createPointFields();

    // Calculate point and row steps
    uint32_t pointStep = hasColors_ ? 16 : 12;
    uint32_t rowStep = width_ * pointStep;

    // Encode point cloud data
    std::string encodedData = pointCloudToBase64();

    // Create JSON payload (sensor_msgs/PointCloud2 format)
    ss << "{";
    ss << "\"header\":{";
    ss << "\"stamp\":{\"sec\":" << sec << ",\"nanosec\":0},";
    ss << "\"frame_id\":\"" << frameId_ << "\"}";
    ss << ",";
    ss << "\"height\":" << height_ << ",";
    ss << "\"width\":" << width_ << ",";
    ss << "\"fields\":[";

    // Add point fields
    for (size_t i = 0; i < fields.size(); ++i) {
        const auto& field = fields[i];
        ss << "{";
        ss << "\"name\":\"" << field.name << "\",";
        ss << "\"offset\":" << field.offset << ",";
        ss << "\"datatype\":" << static_cast<int>(field.datatype) << ",";
        ss << "\"count\":" << field.count;
        ss << "}";
        if (i < fields.size() - 1) {
            ss << ",";
        }
    }

    ss << "],";
    ss << "\"is_bigendian\":false,";
    ss << "\"point_step\":" << pointStep << ",";
    ss << "\"row_step\":" << rowStep << ",";
    ss << "\"data\":\"" << encodedData << "\",";
    ss << "\"is_dense\":true";
    ss << "}";
    // Removed outer rosbridge wrapper

    return ss.str();
} 