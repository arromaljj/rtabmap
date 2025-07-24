#include "RGBImageMessage.h"
#include <iostream>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <ctime>

// For iOS, we'll use a simpler logging system
#define LOG_TAG "RGBImageMessage" // Updated Log Tag
#define LOGI(msg) std::cout << "[INFO][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGW(msg) std::cout << "[WARN][" << LOG_TAG << "] " << (msg) << std::endl
#define LOGE(msg) std::cout << "[ERROR][" << LOG_TAG << "] " << (msg) << std::endl

// Update class name in constructor
RGBImageMessage::RGBImageMessage(const std::string& frameId)
    : Message("", frameId) // Topic is less relevant now, set directly in task
{
}

// Update class name in destructor
RGBImageMessage::~RGBImageMessage()
{
}

// Method name unchanged
void RGBImageMessage::setRGBImage(const cv::Mat& rgb)
{
    // Check that RGB image is valid
    if (rgb.empty() || (rgb.type() != CV_8UC3 && rgb.type() != CV_8UC1)) {
        LOGE("Invalid RGB/Grayscale image. Expected 8UC3 or 8UC1 format");
        return;
    }

    // Store the image
    rgb_ = rgb.clone();

    LOGI("RGB Image set. Size: " + std::to_string(rgb.cols) +
         "x" + std::to_string(rgb.rows) + ", Type: " + std::to_string(rgb.type()));
}

// Method name unchanged
void RGBImageMessage::clear()
{
    rgb_.release();
}

// Helper method unchanged
std::string RGBImageMessage::matToJpegBase64(const cv::Mat& image) const
{
    std::vector<uint8_t> buffer;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 90};
    cv::imencode(".jpg", image, buffer, params);
    return base64_encode(buffer.data(), buffer.size());
}

// Helper method unchanged
std::string RGBImageMessage::compressRGB() const
{
    if (rgb_.empty()) {
        return "";
    }
    // JPEG is better for RGB
    return matToJpegBase64(rgb_);
}

// UPDATED: Renamed method and returns only inner JSON payload
std::string RGBImageMessage::createJsonPayload() const
{
    if (!hasData()) {
        LOGE("Cannot create message payload: no RGB data available");
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

    // Create JSON payload (sensor_msgs/msg/Image format)
    // Note: Escaping backslashes and quotes for JSON within C++ string literal
    ss << "{";
    // Header (ROS2 format)
    ss << "\"header\":{";
    ss << "\"stamp\":{\"sec\":" << sec << ",\"nanosec\":" << nanosec << "},";
    ss << "\"frame_id\":\"" << frameId_ << "\"}";
    ss << ",";
    // Image properties (same in ROS1 and ROS2)
    ss << "\"height\":" << rgb_.rows << ",";
    ss << "\"width\":" << rgb_.cols << ",";
    ss << "\"encoding\":\"" << encoding << "\",";
    ss << "\"is_bigendian\":0,";
    ss << "\"step\":" << step << ",";
    ss << "\"data\":\"" << dataEncoded << "\"}";

    // Close outer JSON (removed rosbridge wrapper)

    LOGI("Created sensor_msgs/msg/Image JSON payload for frame: " + frameId_);
    return ss.str();
} 