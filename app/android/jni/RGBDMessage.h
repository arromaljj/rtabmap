#pragma once
#include "Message.h"
#include <opencv2/core/core.hpp>

// Consider renaming class if it only handles RGB now, e.g., RGBImageMessage
class RGBDMessage : public Message {
public:
    // Update constructor defaults if topic/frameId changed
    RGBDMessage(const std::string& topic = "ios/rgb/image_raw", const std::string& frameId = "camera_link");
    ~RGBDMessage();

    // Changed method signature to only take RGB
    void setRGBImage(const cv::Mat& rgb);
    
    // Create ROS-compatible message (sensor_msgs/Image)
    virtual std::string createRosbridgeMessage() const override;
    
    // Clear data (now only clears RGB)
    void clear();
    
    // Getters
    // Updated hasData to only check rgb_
    bool hasData() const { return !rgb_.empty(); }
    const cv::Mat& getRGB() const { return rgb_; }
    // Removed getDepth()
    // const cv::Mat& getDepth() const { return depth_; }

private:
    cv::Mat rgb_;     // RGB image
    // Removed depth_ member variable
    // cv::Mat depth_;   // Depth image (float or CV_16U)
    
    // Helper methods
    std::string compressRGB() const; // Keep if still used
    // Removed compressDepth() declaration
    // std::string compressDepth() const;
    std::string matToJpegBase64(const cv::Mat& image) const; // Keep if still used
    // Removed matToPngBase64() declaration
    // std::string matToPngBase64(const cv::Mat& image) const;
}; 