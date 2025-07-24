#pragma once
#include "Message.h"
#include <opencv2/core/core.hpp>

// CLASS RENAMED
class RGBImageMessage : public Message {
public:
    // Constructor using default frameId, topic is less relevant now
    RGBImageMessage(const std::string& frameId = "camera_link");
    ~RGBImageMessage();

    // Set RGB image data
    void setRGBImage(const cv::Mat& rgb);

    // UPDATED: Create JSON payload (sensor_msgs/Image format)
    virtual std::string createJsonPayload() const;

    // Clear data
    void clear();

    // Getters
    bool hasData() const { return !rgb_.empty(); }
    const cv::Mat& getRGB() const { return rgb_; }

private:
    cv::Mat rgb_;     // RGB image

    // Helper methods (consider removing if unused)
    std::string compressRGB() const;
    std::string matToJpegBase64(const cv::Mat& image) const;
}; 