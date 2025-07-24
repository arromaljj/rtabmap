#include "RGBDMessage.h"
#include <iostream>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <ctime>

// For iOS, we'll use a simpler logging system
#define LOG_TAG "RGBDMessage"
#define LOGI(msg) std::cout << "[INFO][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGW(msg) std::cout << "[WARN][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGE(msg) std::cout << "[ERROR][" << LOG_TAG << "] " << (msg) << std::endl

RGBDMessage::RGBDMessage(const std::string& topic, const std::string& frameId)
    : Message(topic, frameId)
{
}

RGBDMessage::~RGBDMessage()
{
}

void RGBDMessage::setRGBImage(const cv::Mat& rgb)
{
    // Check that RGB image is valid
    if (rgb.empty() || (rgb.type() != CV_8UC3 && rgb.type() != CV_8UC1)) {
        LOGE("Invalid RGB/Grayscale image. Expected 8UC3 or 8UC1 format");
        return;
    }
    
    // Store the image
    rgb_ = rgb.clone();
    // depth_.release(); // No longer storing depth

    LOGI("RGB Image set. Size: " + std::to_string(rgb.cols) + 
         "x" + std::to_string(rgb.rows) + ", Type: " + std::to_string(rgb.type()));
}

void RGBDMessage::clear()
{
    rgb_.release();
    // depth_.release(); // No longer needed
}

std::string RGBDMessage::matToJpegBase64(const cv::Mat& image) const
{
    std::vector<uint8_t> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    cv::imencode(".jpg", image, buffer, params);
    return base64_encode(buffer.data(), buffer.size());
}

std::string RGBDMessage::compressRGB() const
{
    if (rgb_.empty()) {
        return "";
    }
    // JPEG is better for RGB
    return matToJpegBase64(rgb_);
}

std::string RGBDMessage::createRosbridgeMessage() const
{
    if (!hasData()) {
        LOGE("Cannot create message: no RGB data available");
        return "";
    }
    
    std::stringstream ss;
    
    // Get timestamp - ROS2 uses nanoseconds since epoch
    uint32_t sec = std::time(nullptr);
    uint32_t nanosec = 0; // Ideally would get actual nanoseconds

    // Determine encoding and step based on image type
    std::string encoding;
    uint32_t step = 0;
    if (rgb_.type() == CV_8UC3) {
        encoding = "rgb8"; // Change to rgb8 for ROS2 convention (from bgr8)
        step = static_cast<uint32_t>(rgb_.cols * 3);
    } else if (rgb_.type() == CV_8UC1) {
        encoding = "mono8";
        step = static_cast<uint32_t>(rgb_.cols * 1);
    } else {
         LOGE("Unsupported image type for sensor_msgs/msg/Image: " + std::to_string(rgb_.type()));
         return "";
    }

    // Ensure RGB data is in the right format (OpenCV uses BGR, ROS2 expects RGB)
    cv::Mat rgbForRos;
    if (rgb_.type() == CV_8UC3) {
        cv::cvtColor(rgb_, rgbForRos, cv::COLOR_BGR2RGB);
    } else {
        rgbForRos = rgb_;
    }

    // Base64 encode raw image data
    std::string dataEncoded = base64_encode(rgbForRos.data, rgbForRos.total() * rgbForRos.elemSize());

    // Create JSON message for rosbridge following sensor_msgs/msg/Image format for ROS2
    ss << "{";
    ss << "\\\"op\\\":\\\"publish\\\",";
    ss << "\\\"topic\\\":\\\"" << topic_ << "\\\",";
    ss << "\\\"msg\\\":{";
    
    // Header (ROS2 format)
    ss << "\\\"header\\\":{";
    ss << "\\\"stamp\\\":{\\\"sec\\\":" << sec << ",\\\"nanosec\\\":" << nanosec << "},";
    ss << "\\\"frame_id\\\":\\\"" << frameId_ << "\\\"";
    ss << "},";
    
    // Image properties (same in ROS1 and ROS2)
    ss << "\\\"height\\\":" << rgb_.rows << ",";
    ss << "\\\"width\\\":" << rgb_.cols << ",";
    ss << "\\\"encoding\\\":\\\"" << encoding << "\\\",";
    ss << "\\\"is_bigendian\\\":0,";
    ss << "\\\"step\\\":" << step << ",";
    ss << "\\\"data\\\":\\\"" << dataEncoded << "\\\"";
    
    ss << "}"; // Close msg
    ss << "}"; // Close outer JSON
    
    LOGI("Created sensor_msgs/msg/Image message for topic: " + topic_);
    return ss.str();
} 